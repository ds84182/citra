#[macro_use]
extern crate quick_error;
extern crate mio;
extern crate slab;
extern crate parking_lot;
extern crate iovec;
#[macro_use]
extern crate enum_primitive_derive;
extern crate num_traits;
extern crate socket2;

mod socu;
use socu::hle_unpark::*;

use std::io;
use std::net::{SocketAddr, SocketAddrV4, IpAddr, Ipv4Addr};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Weak};
use std::thread;
use std::mem;
use std::sync::mpsc;
use std::ops::{Deref, DerefMut};

use parking_lot::{RwLock, Mutex};

use mio::net::*;

use slab::Slab;
use iovec::IoVec;

// We use socket2 for asynchronous connect support (since std::net can't do it)
use socket2::{Socket, Domain, Type};

macro_rules! default_ipv4 {
    () => {
        SocketAddrV4::new(Ipv4Addr::new(0, 0, 0, 0), 0)
    };
}

quick_error! {
    #[derive(Debug)]
    pub enum CTRSockError {
        Io(err: io::Error) {
            cause(err)
            from()
        }
        NotSupported
        Canceled
    }
}

impl CTRSockError {
    fn to_ctr_posix_result(&self) -> i32 {
        match self {
            CTRSockError::Io(_err) => {
                -socu::ErrorCode::InputOutput
                // TODO: Implement correctly
            },
            CTRSockError::NotSupported => -socu::ErrorCode::NotSupported,
            CTRSockError::Canceled => -socu::ErrorCode::Canceled,
        }
    }
}

/// A more descriptive version of Rust's option type.
///
/// Essentially contains the result of a synchronous attempt at an
/// asynchronous operation.
#[derive(Debug)]
pub enum Async<T: Sized> {
    /// The HLE thread should park first and retry
    ShouldPark,
    Ready(T)
}

impl<T: Sized> From<Option<T>> for Async<T> {
    fn from(value: Option<T>) -> Async<T> {
        match value {
            Some(v) => Async::Ready(v),
            None => Async::ShouldPark
        }
    }
}

pub type AsyncResult<T, E = CTRSockError> = Result<Async<T>, E>;

/// Trait for a SOCU Socket.
pub trait CTRSock: Sized {
    /// Creates a new CTRSock of the specified type.
    fn new() -> Self;

    /// Connects to the specified address asynchronously.
    ///
    /// A return value of Ok(bool) indicates that the process of connecting
    /// has started successfully, and the boolean indicates whether the connection
    /// is ready (if, for instance, connecting to a socket on the local machine) or
    /// if work will continue asynchronously.
    fn start_connect(&mut self, op: &mut SOCOperations, addr: &SocketAddr, opts: SockOpt) -> AsyncResult<()>;

    /// Continues the connection process asynchronously with an HLE handle.
    ///
    /// Must be called directly after start_connect if it returns Ok(false).
    fn continue_connect_async(&mut self, op: &mut SOCOperations, pending: PendingResultHandle) -> Result<(), CTRSockError>;

    /// Binds the socket for listening on the specified port and address.
    fn bind(&mut self, addr: &SocketAddr) -> Result<(), CTRSockError>;

    /// Starts listening for connections on this socket.
    ///
    /// If this socket is a TCP stream (connect called instead of bind), or
    /// if this socket is UDP-based, this will always return `NotSupported`.
    fn listen(&mut self, op: &mut SOCOperations) -> Result<(), CTRSockError>;

    /// Tries to accept a connection on this socket.
    ///
    /// A return value of Some(i32) indicates either success or failure
    /// (determined by the sign of the i32), and a return value of None
    /// indicates that this operation needs to be processed asynchronously.
    ///
    /// A non-blocking socket will never process operations asynchronously.
    fn try_accept(&self, op: &mut SOCOperations, sockets: &mut SockContainer, opts: SockOpt) -> Async<(i32, SocketAddrV4)>;

    /// Accepts a connection on this socket asynchronously.
    ///
    /// TODO: Change the result to () and complete the result handle with errors encountered synchronously.
    fn accept(&self, op: &mut SOCOperations, pending: PendingResultHandle, opts: SockOpt) -> Result<(), CTRSockError>;

    /// Tries to read data from this socket.
    ///
    /// Return values:
    ///
    /// Ok(None) indicates that this blocking operation should be completed asynchronously.
    ///
    /// Ok(Some(usize)) indicates that the operation was successful.
    ///
    /// Err(i32) indicates that the operation failed, or that this operation would block a non-blocking operation.
    fn try_recv_from(&self, output: &'static mut [u8], opts: SockOpt) -> AsyncResult<usize, i32>;

    fn recv_from(&self, op: &mut SOCOperations, pending: PendingResultHandle, output: &'static mut [u8], opts: SockOpt) -> Result<(), CTRSockError>;
    fn send_to(&self, op: &mut SOCOperations, pending: PendingResultHandle, input: &'static [u8], opts: SockOpt) -> Result<(), CTRSockError>;
}

#[derive(Clone)]
pub struct InFlightRequestCounter(Arc<()>);
pub struct InFlightRequestToken(Weak<()>);

impl InFlightRequestCounter {
    fn new() -> InFlightRequestCounter {
        InFlightRequestCounter(Arc::new(()))
    }

    fn count(&self) -> usize {
        Arc::weak_count(&self.0)
    }

    fn token(&self) -> InFlightRequestToken {
        InFlightRequestToken(Arc::downgrade(&self.0))
    }
}

pub enum CTRSockTcp {
    None,
    Undefined,
    Bound(SocketAddr),
    Listener {
        tcp: Arc<TcpListener>,
        token: mio::Token,
        inflight_accepts: InFlightRequestCounter,
        accepts: mpsc::SyncSender<SOCOpAccept>
    },
    Stream {
        tcp: Arc<TcpStream>,
        token: mio::Token,
        inflight_reads: InFlightRequestCounter,
        reads: mpsc::SyncSender<SOCOpRead>,
        inflight_writes: InFlightRequestCounter,
        writes: mpsc::SyncSender<SOCOpWrite>,
    },
    ConnectNonBlock {
        tcp: Arc<TcpStream>
    },
    ConnectBlockPendingAsync {
        tcp: Arc<TcpStream>
    },
    ConnectBlock {
        tcp: Arc<TcpStream>,
        token: mio::Token,
    },
}

impl CTRSock for CTRSockTcp {
    fn new() -> Self {
        CTRSockTcp::None
    }

    fn start_connect(&mut self, op: &mut SOCOperations, addr: &SocketAddr, opts: SockOpt) -> AsyncResult<()> {
        match self {
            CTRSockTcp::None => {
                let socket = Socket::new(Domain::ipv4(), Type::stream(), None)?;
                socket.set_nonblocking(true)?;
                let res = socket.connect(&(*addr).into());
                match res {
                    Ok(()) => {
                        // Connected immediately!
                        *self = CTRSockTcp::new_stream(TcpStream::from_stream(socket.into())?, op)?;
                        Ok(Async::Ready(()))
                    },
                    Err(ref err) if err.kind() == io::ErrorKind::WouldBlock => {
                        // Connecting async
                        // If the socket is non-blocking, we want to enter connect mode but not have an async task
                        let tcp = Arc::new(TcpStream::from_stream(socket.into())?);
                        if opts.non_blocking {
                            *self = CTRSockTcp::ConnectNonBlock { tcp };
                            Err(CTRSockError::from(io::Error::from(io::ErrorKind::WouldBlock)))
                        } else {
                            *self = CTRSockTcp::ConnectBlockPendingAsync { tcp };
                            Ok(Async::ShouldPark)
                        }
                    },
                    Err(err) => Err(CTRSockError::from(err))
                }
            },
            _ => Err(CTRSockError::NotSupported)
        }
    }

    fn continue_connect_async(&mut self, op: &mut SOCOperations, pending: PendingResultHandle) -> Result<(), CTRSockError> {
        match mem::replace(self, CTRSockTcp::Undefined) {
            CTRSockTcp::ConnectBlockPendingAsync { ref tcp } => {
                let tcp_ref = tcp.clone();
                let token = {
                    let tcp = &*tcp_ref;
                    let token = op.register(tcp, SOCOpQueue::Connect(
                        SOCOpConnectQueue {
                            tcp: Arc::downgrade(&tcp_ref),
                            pending
                        }
                    ))?;
                    op.request_write(tcp, token)?;
                    token
                };
                *self = CTRSockTcp::ConnectBlock {
                    tcp: tcp_ref,
                    token
                };
                Ok(())
            },
            old_self @ _ => {
                *self = old_self;
                Err(CTRSockError::NotSupported)
            }
        }
    }

    fn bind(&mut self, addr: &SocketAddr) -> Result<(), CTRSockError> {
        match self {
            CTRSockTcp::None => {
                *self = CTRSockTcp::Bound(addr.clone());
                Ok(())
            },
            _ => Err(CTRSockError::NotSupported)
        }
    }

    fn listen(&mut self, op: &mut SOCOperations) -> Result<(), CTRSockError> {
        match mem::replace(self, CTRSockTcp::Undefined) {
            CTRSockTcp::Bound(ref addr) => {
                let listener = Arc::new(TcpListener::bind(addr)?);
                let inflight_accepts = InFlightRequestCounter::new();
                let (accepts, accepts_recv) = mpsc::sync_channel(32);
                let token = op.register(
                    listener.deref(),
                    SOCOpQueue::Listener(SOCOpListenerQueue {
                        tcp: Arc::downgrade(&listener),
                        inflight_accepts: inflight_accepts.clone(),
                        accepts: accepts_recv
                    })
                )?;
                *self = CTRSockTcp::Listener {
                    tcp: listener,
                    token,
                    inflight_accepts,
                    accepts,
                };
                Ok(())
            },
            old_self @ _ => {
                *self = old_self;
                Err(CTRSockError::NotSupported)
            }
        }
    }

    fn try_accept(&self, op: &mut SOCOperations, sockets: &mut SockContainer, opts: SockOpt) -> Async<(i32, SocketAddrV4)> {
        match self {
            CTRSockTcp::Listener { ref tcp, ref inflight_accepts, .. } => {
                if inflight_accepts.count() > 0 {
                    // Some accepts are in the queue, don't steal their stuff
                    if opts.non_blocking {
                        Async::Ready((
                            -socu::ErrorCode::WouldBlock,
                            default_ipv4!()
                        ))
                    } else {
                        Async::ShouldPark
                    }
                } else {
                    match tcp.accept() {
                        Ok((socket, SocketAddr::V4(addr))) => {
                            match CTRSockTcp::new_stream(socket, op) {
                                Ok(sock) => Async::from(sockets.adopt(sock).map(|s| (s as i32, addr))),
                                Err(err) => Async::Ready((
                                    CTRSockError::from(err).to_ctr_posix_result(),
                                    default_ipv4!()
                                ))
                            }
                        },
                        // If the OS accepts an IPv6 connection on an IPv4 TCP Socket then we have bigger problems
                        Ok(..) => unreachable!(),
                        Err(ref err) if err.kind() == io::ErrorKind::WouldBlock => {
                            if opts.non_blocking {
                                Async::Ready((
                                    -socu::ErrorCode::WouldBlock,
                                    default_ipv4!()
                                ))
                            } else {
                                Async::ShouldPark
                            }
                        },
                        Err(err) => Async::Ready((
                            CTRSockError::from(err).to_ctr_posix_result(),
                            default_ipv4!()
                        ))
                    }
                }
            },
            _ => Async::Ready((
                -socu::ErrorCode::NotSupported,
                default_ipv4!()
            ))
        }
    }

    fn accept(&self, op: &mut SOCOperations, pending: PendingResultHandle, opts: SockOpt) -> Result<(), CTRSockError> {
        match self {
            CTRSockTcp::Listener { ref tcp, ref inflight_accepts, ref accepts, token, .. } => {
                op.request_accept(tcp.deref(), *token)?;
                accepts.send(SOCOpAccept {
                    _ref: inflight_accepts.token(),
                    pending,
                }).unwrap();

                Ok(())
            },
            _ => Err(CTRSockError::NotSupported)
        }
    }

    fn try_recv_from(&self, output: &'static mut [u8], opts: SockOpt) -> AsyncResult<usize, i32> {
        match self {
            CTRSockTcp::Stream { ref tcp, ref inflight_reads, .. } => {
                if inflight_reads.count() > 0 {
                    // Some reads are in the queue, don't steal their stuff
                    if opts.non_blocking {
                        Err(-socu::ErrorCode::WouldBlock)
                    } else {
                        Ok(Async::ShouldPark)
                    }
                } else {
                    let res = tcp.read_bufs(&mut [IoVec::from_bytes_mut(output).unwrap()]);

                    match res {
                        Ok(n) => {
                            println!("Read {} bytes synchronously", n);
                            Ok(Async::Ready(n))
                        },
                        Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => {
                            if opts.non_blocking {
                                Err(-socu::ErrorCode::WouldBlock)
                            } else {
                                Ok(Async::ShouldPark)
                            }
                        },
                        Err(e) => Err(CTRSockError::from(e).to_ctr_posix_result())
                    }
                }
            },
            _ => Err(-socu::ErrorCode::NotSupported)
        }
    }

    fn recv_from(&self, op: &mut SOCOperations, pending: PendingResultHandle, output: &'static mut [u8], opts: SockOpt) -> Result<(), CTRSockError> {
        match self {
            CTRSockTcp::Stream { ref tcp, ref inflight_reads, ref reads, token, .. } => {
                op.request_read(tcp.deref(), *token)?;
                reads.send(SOCOpRead {
                    _ref: inflight_reads.token(),
                    pending,
                    read_bytes: 0,
                    output
                }).unwrap();
                Ok(())
            },
            _ => Err(CTRSockError::NotSupported)
        }
    }

    fn send_to(&self, op: &mut SOCOperations, pending: PendingResultHandle, input: &'static [u8], opts: SockOpt) -> Result<(), CTRSockError> {
        match self {
            CTRSockTcp::Stream { ref tcp, ref inflight_writes, ref writes, token, .. } => {
                op.request_write(tcp.deref(), *token)?;
                writes.send(SOCOpWrite {
                    _ref: inflight_writes.token(),
                    pending,
                    write_bytes: 0,
                    input
                }).unwrap();
                Ok(())
            },
            _ => Err(CTRSockError::NotSupported)
        }
    }
}

impl CTRSockTcp {
    fn new_stream(sock: TcpStream, op: &mut SOCOperations) -> io::Result<CTRSockTcp> {
        let sock = Arc::new(sock);
        let inflight_reads = InFlightRequestCounter::new();
        let (reads, reads_recv) = mpsc::sync_channel(32);
        let inflight_writes = InFlightRequestCounter::new();
        let (writes, writes_recv) = mpsc::sync_channel(32);
        let token = op.register(
            sock.deref(),
            SOCOpQueue::Stream(SOCOpStreamQueue {
                tcp: Arc::downgrade(&sock),
                inflight_reads: inflight_reads.clone(),
                current_read: None,
                reads: reads_recv,
                inflight_writes: inflight_writes.clone(),
                current_write: None,
                writes: writes_recv,
                requesting: mio::Ready::empty(),
            })
        )?;
        Ok(CTRSockTcp::Stream {
            tcp: sock,
            token,
            inflight_reads,
            reads,
            inflight_writes,
            writes
        })
    }
}

macro_rules! array {
    (@accum (0, $($_es:expr),*) -> ($($body:tt)*))
        => {array!(@as_expr [$($body)*])};
    (@accum (2, $($es:expr),*) -> ($($body:tt)*))
        => {array!(@accum (0, $($es),*) -> ($($body)* $($es,)* $($es,)*))};
    (@accum (4, $($es:expr),*) -> ($($body:tt)*))
        => {array!(@accum (2, $($es,)* $($es),*) -> ($($body)*))};
    (@accum (8, $($es:expr),*) -> ($($body:tt)*))
        => {array!(@accum (4, $($es,)* $($es),*) -> ($($body)*))};
    (@accum (16, $($es:expr),*) -> ($($body:tt)*))
        => {array!(@accum (8, $($es,)* $($es),*) -> ($($body)*))};
    (@accum (32, $($es:expr),*) -> ($($body:tt)*))
        => {array!(@accum (16, $($es,)* $($es),*) -> ($($body)*))};
    (@accum (64, $($es:expr),*) -> ($($body:tt)*))
        => {array!(@accum (32, $($es,)* $($es),*) -> ($($body)*))};

    (@as_expr $e:expr) => {$e};

    [$e:expr; $n:tt] => { array!(@accum ($n, $e) -> ()) };
}

struct AtomicSockOpt {
    non_blocking: AtomicBool
}

#[derive(Clone)]
pub struct SockOpt {
    non_blocking: bool
}

impl AtomicSockOpt {
    fn new() -> AtomicSockOpt {
        AtomicSockOpt {
            non_blocking: AtomicBool::new(false),
        }
    }

    fn set_non_blocking(&self, blocking: bool) {
        self.non_blocking.store(blocking, Ordering::Release);
    }

    fn get_non_blocking(&self) -> bool {
        self.non_blocking.load(Ordering::Acquire)
    }
}

impl<'a> From<&'a AtomicSockOpt> for SockOpt {
    fn from(value: &'a AtomicSockOpt) -> SockOpt {
        SockOpt {
            non_blocking: value.get_non_blocking()
        }
    }
}

const SOC_COUNT: usize = 64;

type SocData = (RwLock<CTRSockTcp>, AtomicSockOpt);

pub struct SockContainer {
    sockets: [(Option<Arc<SocData>>, u16); SOC_COUNT]
}

impl SockContainer {
    fn new() -> SockContainer {
        SockContainer {
            sockets: array![(None, 0); 64]
        }
    }

    fn adopt(&mut self, adopt_sock: CTRSockTcp) -> Option<u32> {
        let index = self.sockets.iter().position(|(sock, _)| sock.is_none())?;
        let base = index as u32;
        let (ref mut sock, seq) = &mut self.sockets[index];
        *sock = Some(Arc::new((RwLock::new(adopt_sock), AtomicSockOpt::new())));
        Some(base | ((*seq as u32) << 8))
    }

    fn new_socket(&mut self) -> Option<u32> {
        self.adopt(CTRSockTcp::new())
    }

    fn socket_at(&self, id: i32) -> Option<Arc<SocData>> {
        if id < 0 {
            return None;
        }
        let base = (id & 0xFF) as usize;
        let seq = ((id & 0xFFFF00) >> 8) as u16;
        if base > SOC_COUNT {
            return None
        }
        let (ref sock, actual_seq) = &self.sockets[base];
        if seq != *actual_seq {
            return None
        }
        sock.clone()
    }

    fn close_socket(&mut self, id: i32) -> bool {
        if id < 0 {
            return false;
        }
        let base = (id & 0xFF) as usize;
        let seq = ((id & 0xFFFF00) >> 8) as u16;
        if base > SOC_COUNT {
            return false;
        }
        if seq != self.sockets[base].1 {
            false
        } else {
            let (ref mut sock, actual_seq) = &mut self.sockets[base];
            *actual_seq = actual_seq.wrapping_add(1);
            *sock = None;
            true
        }
    }
}

#[derive(Debug)]
enum SOCResult {
    Allocated,
    Pending(&'static mut HLEResumeToken),
    Ok(i32),
    AcceptOk(i32, SocketAddrV4),
    ReadWriteOk(i32, usize),
    Sentinel
}

pub struct SOCResultContainer(Slab<SOCResult>);

impl SOCResultContainer {
    fn new() -> SOCResultContainer {
        SOCResultContainer(Slab::new())
    }

    fn complete_unpark<F>(cont: &RwLock<SOCResultContainer>, func: F)
        where F: FnOnce(&mut SOCResultContainer) -> Option<HLEResumeTokenRef> {
        let mut guard = cont.write();
        func(&mut *guard).try_unpark(guard);
    }

    fn allocate(&mut self) -> usize {
        self.0.insert(SOCResult::Allocated)
    }

    fn start_operation(&mut self, index: usize, token: HLEResumeTokenRef) {
        let res = &mut self.0[index];

        match mem::replace(res, SOCResult::Sentinel) {
            SOCResult::Allocated => mem::replace(res, SOCResult::Pending(token)),
            _ => panic!("Cannot start operation on used result!")
        };
    }

    #[must_use]
    fn complete(&mut self, index: usize, result: i32) -> Option<HLEResumeTokenRef> {
        match mem::replace(&mut self.0[index], SOCResult::Ok(result)) {
            SOCResult::Pending(hle_token) => Some(hle_token),
            _ => None
        }
    }

    #[must_use]
    fn complete_accept(&mut self, index: usize, result: i32, addr: SocketAddrV4) -> Option<HLEResumeTokenRef> {
        match mem::replace(&mut self.0[index], SOCResult::AcceptOk(result, addr)) {
            SOCResult::Pending(hle_token) => Some(hle_token),
            _ => None
        }
    }

    #[must_use]
    fn complete_read(&mut self, index: usize, result: i32, size: usize) -> Option<HLEResumeTokenRef> {
        match mem::replace(&mut self.0[index], SOCResult::ReadWriteOk(result, size)) {
            SOCResult::Pending(hle_token) => Some(hle_token),
            _ => None
        }
    }

    fn consume(&mut self, index: usize) -> Option<SOCResult> {
        if self.0.contains(index) {
            Some(self.0.remove(index))
        } else {
            None
        }
    }
}

trait SOCResultComplete {
    fn complete(&self, index: usize, result: i32);
    fn complete_accept(&self, index: usize, result: i32, addr: SocketAddrV4);
    fn complete_read(&self, index: usize, result: i32, size: usize);
}

impl SOCResultComplete for RwLock<SOCResultContainer> {
    fn complete(&self, index: usize, result: i32) {
        SOCResultContainer::complete_unpark(
            self,
            |r| r.complete(index, result)
        );
    }

    fn complete_accept(&self, index: usize, result: i32, addr: SocketAddrV4) {
        SOCResultContainer::complete_unpark(
            self,
            |r| r.complete_accept(index, result, addr)
        );
    }

    fn complete_read(&self, index: usize, result: i32, size: usize) {
        SOCResultContainer::complete_unpark(
            self,
            |r| r.complete_read(index, result, size)
        );
    }
}

pub struct PendingResultHandle {
    index: usize
}

impl PendingResultHandle {
    fn new(index: usize) -> PendingResultHandle {
        PendingResultHandle { index }
    }
}

pub struct SOCOpAccept {
    _ref: InFlightRequestToken,
    pending: PendingResultHandle
}

pub struct SOCOpRead {
    _ref: InFlightRequestToken,
    pending: PendingResultHandle,
    output: &'static mut [u8],
    read_bytes: usize
}

pub struct SOCOpWrite {
    _ref: InFlightRequestToken,
    pending: PendingResultHandle,
    input: &'static [u8],
    write_bytes: usize
}

struct SOCOpListenerQueue {
    tcp: Weak<TcpListener>,
    inflight_accepts: InFlightRequestCounter,
    accepts: mpsc::Receiver<SOCOpAccept>
}

impl SOCOpListenerQueue {
    fn ref_tcp(&self) -> Arc<TcpListener> {
        self.tcp.upgrade().expect("Invalid state for listener queue")
    }

    fn has_inflight_accept(&self) -> bool {
        self.inflight_accepts.count() != 0
    }

    fn accept(&self) -> io::Result<(TcpStream, SocketAddr)> {
        self.ref_tcp().accept()
    }

    fn next_accept(&self) -> SOCOpAccept {
        self.accepts.try_recv().expect("Invalid state for listener queue")
    }

    fn reschedule(&self, ops: &SOCOperations, token: mio::Token) {
        // Try to reschedule
        if self.inflight_accepts.count() != 0 {
            let tcp_ref = self.ref_tcp();
            let tcp = tcp_ref.deref();
            ops.request_accept(tcp, token).expect("Failed to enqueue TCP listener");
        }
    }
}

struct SOCOpStreamQueue {
    tcp: Weak<TcpStream>,
    inflight_reads: InFlightRequestCounter,
    current_read: Option<SOCOpRead>,
    reads: mpsc::Receiver<SOCOpRead>,
    inflight_writes: InFlightRequestCounter,
    current_write: Option<SOCOpWrite>,
    writes: mpsc::Receiver<SOCOpWrite>,
    requesting: mio::Ready,
}

impl SOCOpStreamQueue {
    fn ref_tcp(&self) -> Arc<TcpStream> {
        self.tcp.upgrade().expect("Invalid state for listener queue")
    }
}

struct SOCOpConnectQueue {
    tcp: Weak<TcpStream>,
    pending: PendingResultHandle,
}

enum SOCOpQueueType {
    Listener, Stream, Connect
}

enum SOCOpQueue {
    Listener(SOCOpListenerQueue),
    Stream(SOCOpStreamQueue),
    Connect(SOCOpConnectQueue)
}

impl SOCOpQueue {
    fn queue_type(&self) -> SOCOpQueueType {
        match self {
            SOCOpQueue::Listener(..) => SOCOpQueueType::Listener,
            SOCOpQueue::Stream(..) => SOCOpQueueType::Stream,
            SOCOpQueue::Connect(..) => SOCOpQueueType::Connect,
        }
    }
}

pub struct SOCOperations {
    poll: Arc<mio::Poll>,
    slab: Slab<Mutex<SOCOpQueue>>
}

impl SOCOperations {
    fn new(poll: Arc<mio::Poll>) -> SOCOperations {
        SOCOperations {
            poll,
            slab: Slab::new()
        }
    }

    fn queue_at(&self, token: mio::Token) -> &Mutex<SOCOpQueue> {
        &self.slab[token.0]
    }

    fn register<E: mio::Evented>(&mut self, evented: &E, queue: SOCOpQueue) -> io::Result<mio::Token> {
        let entry = self.slab.vacant_entry();
        let token = mio::Token(entry.key());

        self.poll.register(evented, token, mio::Ready::empty(), mio::PollOpt::empty())?;
        entry.insert(Mutex::new(queue));

        Ok(token)
    }

    fn request_accept<E: mio::Evented>(&self, evented: &E, token: mio::Token) -> io::Result<()> {
        self.poll.reregister(evented, token, mio::Ready::readable(), mio::PollOpt::oneshot() | mio::PollOpt::edge())
    }

    fn request_read<E: mio::Evented>(&self, evented: &E, token: mio::Token) -> io::Result<()> {
        let mut queue = self.queue_at(token).lock();
        if let &mut SOCOpQueue::Stream(SOCOpStreamQueue { ref mut requesting, .. }) = queue.deref_mut() {
            if requesting.is_readable() {
                // Nothing to do
                Ok(())
            } else {
                *requesting |= mio::Ready::readable();
                self.poll.reregister(evented, token, *requesting, mio::PollOpt::oneshot() | mio::PollOpt::edge())
            }
        } else {
            panic!("Cannot request to read something that isn't a stream!")
        }
    }

    fn request_write<E: mio::Evented>(&self, evented: &E, token: mio::Token) -> io::Result<()> {
        let mut queue = self.queue_at(token).lock();
        if let &mut SOCOpQueue::Stream(SOCOpStreamQueue { ref mut requesting, .. }) = queue.deref_mut() {
            if requesting.is_writable() {
                // Nothing to do
                Ok(())
            } else {
                *requesting |= mio::Ready::writable();
                self.poll.reregister(evented, token, *requesting, mio::PollOpt::oneshot() | mio::PollOpt::edge())
            }
        } else {
            panic!("Cannot request to read something that isn't a stream!")
        }
    }
}

pub struct SOCUContext {
    poll: Arc<mio::Poll>,
    sockets: Arc<RwLock<SockContainer>>,
    results: Arc<RwLock<SOCResultContainer>>,
    ops: Arc<RwLock<SOCOperations>>,
}

impl SOCUContext {
    fn new() -> io::Result<SOCUContext> {
        let poll = Arc::new(mio::Poll::new()?);
        let ctx = SOCUContext {
            poll: poll.clone(),
            sockets: Arc::new(RwLock::new(SockContainer::new())),
            results: Arc::new(RwLock::new(SOCResultContainer::new())),
            ops: Arc::new(RwLock::new(SOCOperations::new(poll))),
        };

        let poll = ctx.poll.clone();
        let ops = ctx.ops.clone();
        let results = ctx.results.clone();
        let sockets = ctx.sockets.clone();
        thread::spawn(move || {
            let mut events = mio::Events::with_capacity(1024);
            loop {
                poll.poll(&mut events, None).unwrap();

                let mut ops_guard = ops.write();

                for event in &events {
                    println!("{:?}", event);
                    let token = event.token();
                    let queue_type = ops_guard.queue_at(token).lock().queue_type();

                    match queue_type {
                        SOCOpQueueType::Listener => {
                            let res = if let SOCOpQueue::Listener(queue) = ops_guard.queue_at(token).lock().deref() {
                                if !queue.has_inflight_accept() {
                                    None
                                } else {
                                    Some(queue.accept())
                                }
                            } else {
                                unreachable!()
                            };

                            if let Some(res) = res {
                                let posix_result = match res {
                                    Ok((socket, SocketAddr::V4(addr))) => {
                                        let res = match CTRSockTcp::new_stream(socket, ops_guard.deref_mut()) {
                                            Ok(sock) => sockets.write().adopt(sock).map(|s| (s as i32, addr)),
                                            Err(err) => Some((
                                                CTRSockError::from(err).to_ctr_posix_result(),
                                                default_ipv4!()
                                            ))
                                        };
                                        res
                                    },
                                    // If the OS accepts an IPv6 connection on an IPv4 TCP Socket then we have bigger problems
                                    Ok(..) => unreachable!(),
                                    Err(ref err) if err.kind() == io::ErrorKind::WouldBlock => None,
                                    Err(err) => Some((
                                        CTRSockError::from(err).to_ctr_posix_result(),
                                        default_ipv4!()
                                    ))
                                };

                                if let SOCOpQueue::Listener(queue) = ops_guard.queue_at(token).lock().deref() {
                                    if let Some((posix_result, addr)) = posix_result {
                                        let accept_op = queue.next_accept();
                                        results.complete_accept(accept_op.pending.index, posix_result, addr);
                                    }

                                    queue.reschedule(ops_guard.deref(), token);
                                } else {
                                    unreachable!()
                                }
                            }
                        },
                        SOCOpQueueType::Stream => {
                            // Either process current read or take next read
                            let mut queue_guard = ops_guard.queue_at(token).lock();
                            let mut stream = if let SOCOpQueue::Stream(ref mut stream) = queue_guard.deref_mut() {
                                stream
                            } else {
                                unreachable!()
                            };

                            if event.readiness().is_readable() {
                                let mut tcp = stream.tcp.upgrade().expect("Invalid state for listener queue");

                                loop {
                                    // Only 2 reasons for read events to be enabled:
                                    // There is an ongoing read OR
                                    // There is a new read in the queue
                                    let discard = {
                                        let mut current_read = {
                                            if let Some(ref mut current_read) = stream.current_read {
                                                current_read
                                            } else {
                                                let read = stream.reads.try_recv().unwrap();
                                                stream.current_read = Some(read);
                                                stream.current_read.as_mut().unwrap()
                                            }
                                        };

                                        // Start processing the read
                                        let res = tcp.read_bufs(&mut [IoVec::from_bytes_mut(current_read.output).unwrap()]);

                                        match res {
                                            Ok(n) => {
                                                println!("Read {} bytes", n);
                                                current_read.read_bytes += n;

                                                results.complete_read(current_read.pending.index, current_read.read_bytes as i32, current_read.read_bytes);
                                                true

                                                // TODO: Use below for MSG_WAITALL
                                                /*
                                                if n == 0 || n == current_read.output.len() {
                                                    // Stream has closed or entire buffer has been read
                                                    let token = results.write()
                                                        .complete_read(current_read.pending.index, current_read.read_bytes as i32, current_read.read_bytes);
                                                    token.try_unpark();
                                                    true
                                                } else {
                                                    let output = mem::replace(&mut current_read.output, &mut []);
                                                    current_read.output = &mut output[n..];
                                                    false
                                                }
                                                */
                                            },
                                            Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => {
                                                println!("Would block!");
                                                if current_read.read_bytes == 0 {
                                                    // Haven't read any bytes, can't unpark + dispose
                                                    false
                                                } else {
                                                    results.complete_read(current_read.pending.index, current_read.read_bytes as i32, current_read.read_bytes);
                                                    true
                                                }
                                            },
                                            Err(e) => {
                                                results.complete_read(current_read.pending.index, CTRSockError::from(e).to_ctr_posix_result(), 0);
                                                true
                                            }
                                        }
                                    };

                                    if discard {
                                        stream.current_read = None;
                                        if let Ok(recv) = stream.reads.try_recv() {
                                            stream.current_read = Some(recv);
                                        } else {
                                            break
                                        }
                                    } else {
                                        break
                                    }
                                }

                                if stream.current_read.is_some() {
                                    poll.reregister(tcp.deref(), token, stream.requesting, mio::PollOpt::oneshot() | mio::PollOpt::edge()).unwrap();
                                } else {
                                    stream.requesting = stream.requesting ^ mio::Ready::readable();
                                }
                            } else if event.readiness().is_writable() {
                                let mut tcp = stream.tcp.upgrade().expect("Invalid state for listener queue");

                                loop {
                                    // Only 2 reasons for read events to be enabled:
                                    // There is an ongoing read OR
                                    // There is a new read in the queue
                                    let discard = {
                                        let mut current_write = {
                                            if let Some(ref mut current_write) = stream.current_write {
                                                current_write
                                            } else {
                                                let write = stream.writes.try_recv().unwrap();
                                                stream.current_write = Some(write);
                                                stream.current_write.as_mut().unwrap()
                                            }
                                        };

                                        // Start processing the write
                                        let res = tcp.write_bufs(&[IoVec::from_bytes(current_write.input).unwrap()]);

                                        match res {
                                            Ok(n) => {
                                                println!("Wrote {} bytes", n);
                                                current_write.write_bytes += n;

                                                results.complete_read(current_write.pending.index, current_write.write_bytes as i32, current_write.write_bytes);
                                                true

                                                // TODO: Use below for MSG_WAITALL
                                                /*
                                                if n == 0 || n == current_write.input.len() {
                                                    // Haven't read any bytes, can't unpark + dispose
                                                    let token = results.write()
                                                        .complete_read(current_write.pending.index, current_write.write_bytes as i32, current_write.write_bytes);
                                                    token.try_unpark();
                                                    true
                                                } else {
                                                    current_write.input = &current_write.input[n..];
                                                    false
                                                }
                                                */
                                            },
                                            Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => {
                                                println!("Would block!");
                                                if current_write.write_bytes == 0 {
                                                    // Haven't written any bytes, can't unpark + dispose
                                                    false
                                                } else {
                                                    results.complete_read(current_write.pending.index, current_write.write_bytes as i32, current_write.write_bytes);
                                                    true
                                                }
                                            },
                                            Err(e) => {
                                                results.complete_read(current_write.pending.index, CTRSockError::from(e).to_ctr_posix_result(), 0);
                                                true
                                            }
                                        }
                                    };

                                    if discard {
                                        stream.current_write = None;
                                        if let Ok(recv) = stream.writes.try_recv() {
                                            stream.current_write = Some(recv);
                                        } else {
                                            break
                                        }
                                    } else {
                                        break
                                    }
                                }

                                if stream.current_write.is_some() {
                                    poll.reregister(tcp.deref(), token, stream.requesting, mio::PollOpt::oneshot() | mio::PollOpt::edge()).unwrap();
                                } else {
                                    stream.requesting = stream.requesting ^ mio::Ready::writable();
                                }
                            }
                        },
                        SOCOpQueueType::Connect => {
                            if let SOCOpQueue::Connect(queue) = ops_guard.queue_at(token).lock().deref() {
                                let tcp = queue.tcp.upgrade().expect("Invalid state for listener queue");
                                match tcp.take_error() {
                                    Ok(None) => {
                                        panic!("Ok, the socket connected.");
                                    },
                                    Ok(Some(err)) => {
                                        panic!("Ok, the socket didn't connect? {:?}", err);
                                    },
                                    Err(err) => {
                                        panic!("Ok, we failed to get the error? {:?}", err);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        });

        Ok(ctx)
    }
}

#[no_mangle]
pub extern fn citrs_socu_init() -> *mut SOCUContext {
    Box::into_raw(Box::new(SOCUContext::new().unwrap()))
}

#[no_mangle]
pub extern fn citrs_socu_socket(ctx: &mut SOCUContext) -> i32 {
    if let Some(soc) = ctx.sockets.write().new_socket() {
        soc as i32
    } else {
        std::i32::MIN
    }
}

fn deref_opt<'a, T: Deref + 'a>(opt: &'a Option<T>) -> Option<&'a T::Target> {
    if let Some(v) = opt {
        Some(v.deref())
    } else {
        None
    }
}

#[no_mangle]
pub extern fn citrs_socu_set_non_blocking(ctx: &mut SOCUContext, sock: i32, non_blocking: bool) -> i32 {
    if let Some((_sock, opt)) = deref_opt(&ctx.sockets.read().socket_at(sock)) {
        opt.set_non_blocking(non_blocking);
        0
    } else {
        -socu::ErrorCode::BadFD
    }
}

#[no_mangle]
pub extern fn citrs_socu_get_non_blocking(ctx: &mut SOCUContext, sock: i32, non_blocking: &mut bool) -> i32 {
    if let Some((_sock, opt)) = deref_opt(&ctx.sockets.read().socket_at(sock)) {
        *non_blocking = opt.get_non_blocking();
        0
    } else {
        -socu::ErrorCode::BadFD
    }
}

#[no_mangle]
pub extern fn citrs_socu_bind(ctx: &mut SOCUContext, sock: i32, address: u32, port: u16) -> i32 {
    if let Some((sock, _opt)) = deref_opt(&ctx.sockets.read().socket_at(sock)) {
        // Assemble IP Address
        let address = Ipv4Addr::new(
            ((address & 0x000000FF) >>  0) as u8,
            ((address & 0x0000FF00) >>  8) as u8,
            ((address & 0x00FF0000) >> 16) as u8,
            ((address & 0xFF000000) >> 24) as u8,
        );

        let address = SocketAddr::new(IpAddr::V4(address), port);

        match sock.write().bind(&address) {
            Ok(()) => 0,
            Err(err) => err.to_ctr_posix_result(),
        }
    } else {
        -socu::ErrorCode::BadFD
    }
}

#[no_mangle]
pub extern fn citrs_socu_listen(ctx: &mut SOCUContext, sock: i32) -> i32 {
    if let Some((sock, _opt)) = deref_opt(&ctx.sockets.write().socket_at(sock)) {
        match sock.write().listen(&mut ctx.ops.write()) {
            Ok(()) => 0,
            Err(err) => err.to_ctr_posix_result(),
        }
    } else {
        -socu::ErrorCode::BadFD
    }
}

#[no_mangle]
pub extern fn citrs_socu_prepare_async_result(ctx: &mut SOCUContext) -> usize {
    ctx.results.write().allocate()
}

#[no_mangle]
pub extern fn citrs_socu_try_accept(ctx: &mut SOCUContext, sock: i32, posix_result: &mut i32, ipv4: &mut [u8; 4], port: &mut u16) -> bool {
    let mut socks_wg = ctx.sockets.write();
    if let Some((sock, opt)) = deref_opt(&socks_wg.socket_at(sock)) {
        match sock.read().try_accept(&mut ctx.ops.write(), &mut socks_wg, opt.into()) {
            Async::Ready((res, addr)) => {
                *posix_result = res;
                *ipv4 = addr.ip().octets();
                *port = addr.port();
                true
            },
            Async::ShouldPark => false
        }
    } else {
        *posix_result = -socu::ErrorCode::BadFD;
        true
    }
}

#[no_mangle]
pub extern fn citrs_socu_accept_async(ctx: &mut SOCUContext, sock: i32, token: &'static mut HLEResumeToken, result: usize) {
    ctx.results.write().start_operation(result, token);
    if let Some((sock, opt)) = deref_opt(&ctx.sockets.write().socket_at(sock)) {
        let pending = PendingResultHandle::new(result);
        match sock.read().accept(&mut ctx.ops.write(), pending, opt.into()) {
            Ok(..) => {},
            Err(err) => {
                // We can't unpark on the current thread else Citra breaks
                let results = ctx.results.clone();
                thread::spawn(move || {
                    results.complete(result, err.to_ctr_posix_result());
                });
            },
        }
    } else {
        panic!("Please call citrs_socu_try_accept before citrs_socu_accept_async (socket not found)")
    }
}

#[no_mangle]
pub extern fn citrs_socu_try_recv(ctx: &mut SOCUContext, sock: i32, output_ptr: *mut u8, output_len: usize, posix_result: &mut i32, read_amount: &mut usize) -> bool {
    let output = unsafe { std::slice::from_raw_parts_mut(output_ptr, output_len) };
    if let Some((sock, opt)) = deref_opt(&ctx.sockets.write().socket_at(sock)) {
        match sock.read().try_recv_from(output, opt.into()) {
            Ok(Async::Ready(n)) => {
                *posix_result = 0;
                *read_amount = n;
                true
            },
            Ok(Async::ShouldPark) => false,
            Err(e) => {
                *posix_result = e;
                true
            }
        }
    } else {
        *posix_result = -socu::ErrorCode::BadFD;
        true
    }
}

#[no_mangle]
pub extern fn citrs_socu_recv(ctx: &mut SOCUContext, sock: i32, token: &'static mut HLEResumeToken, result: usize, output_ptr: *mut u8, output_len: usize) {
    let output = unsafe { std::slice::from_raw_parts_mut(output_ptr, output_len) };
    ctx.results.write().start_operation(result, token);
    if let Some((sock, opt)) = deref_opt(&ctx.sockets.write().socket_at(sock)) {
        let pending = PendingResultHandle::new(result);
        match sock.read().recv_from(&mut ctx.ops.write(), pending, output, opt.into()) {
            Ok(()) => {},
            Err(err) => {
                ctx.results.complete(result, err.to_ctr_posix_result());
            },
        }
    } else {
        panic!("Please call citrs_socu_try_recv before citrs_socu_recv (socket not found)")
    }
}

#[no_mangle]
pub extern fn citrs_socu_send(ctx: &mut SOCUContext, sock: i32, token: &'static mut HLEResumeToken, result: usize, input_ptr: *const u8, input_len: usize) {
    let input = unsafe { std::slice::from_raw_parts(input_ptr, input_len) };
    ctx.results.write().start_operation(result, token);
    if let Some((sock, opt)) = deref_opt(&ctx.sockets.write().socket_at(sock)) {
        let pending = PendingResultHandle::new(result);
        match sock.read().send_to(&mut ctx.ops.write(), pending, input, opt.into()) {
            Ok(()) => {},
            Err(err) => {
                ctx.results.complete(result, err.to_ctr_posix_result());
            },
        }
    } else {
        ctx.results.complete(result, -socu::ErrorCode::BadFD);
    }
}

#[no_mangle]
pub extern fn citrs_socu_close(ctx: &mut SOCUContext, sock: i32) -> i32 {
    if ctx.sockets.write().close_socket(sock) {
        0
    } else {
        -socu::ErrorCode::BadFD
    }
}

#[no_mangle]
pub extern fn citrs_socu_consume(ctx: &mut SOCUContext, result: usize) -> i32 {
    match ctx.results.write().consume(result) {
        Some(SOCResult::Ok(res)) => res,
        res @ _ => panic!("Cannot consume result in this state ({:?})", res)
    }
}

#[no_mangle]
pub extern fn citrs_socu_consume_accept(ctx: &mut SOCUContext, result: usize, ipv4: &mut [u8; 4], port: &mut u16) -> i32 {
    match ctx.results.write().consume(result) {
        Some(SOCResult::AcceptOk(res, addr)) => {
            *ipv4 = addr.ip().octets();
            *port = addr.port();
            res
        },
        res @ _ => panic!("Cannot consume result in this state ({:?})", res)
    }
}

#[no_mangle]
pub extern fn citrs_socu_consume_read(ctx: &mut SOCUContext, result: usize, read_amount: &mut usize) -> i32 {
    match ctx.results.write().consume(result) {
        Some(SOCResult::ReadWriteOk(res, size)) => {
            *read_amount = size;
            res
        },
        res @ _ => panic!("Cannot consume result in this state ({:?})", res)
    }
}

#[no_mangle]
pub extern fn citrs_socu_exit(ctx: *mut SOCUContext) {
    unsafe { Box::from_raw(ctx) };
}

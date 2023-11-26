"""
TCP Socket
For creation and monitoring of the TCP socket
"""
from ipaddress import ip_address
import socket
import errno
import types
from threading import Thread
from typing import Dict, Any, Tuple, TypeVar, Callable
from selectors import (
    DefaultSelector,
    EVENT_READ,
    EVENT_WRITE,
    SelectorKey
)


ArgT = TypeVar("ArgT")
ReturnT = TypeVar("ReturnT")


class TCPSocketServer:
    """
    TCP Socket server
    ...

    Attributes
    ----------
    CONNECTED_CLIENTS:Dict[str, socket.socket]
        dict of connected clients.
    TERMINATE: bool
        Terminating connection

    __host: str
        TCP host
    __port: int
        TCP port
    __select: DefaultSelector
        Event Selector
    __operations: Dict[int, Any]
        user requested operations


    Methods
    -------
    HOST(self) -> str
        property for setting or getting TCP host
    PORT(self) -> int:
        property for setting or getting TCP port
    operations(self) -> Dict[int, Any]:
        property for setting or getting operations
    __cleanup_connections(self) -> None:
        Cleaning up connections before terminating.
    __accept_wrapper(self, sock: socket.socket) -> None:
        Accept new incoming connections.
    __process_request(self, request: bytes, sock) -> None:
        Mapping client request to registered process
    __service_connection(self, key: SelectorKey, mask: int) -> None:
        Handle client messages.
    start_server(self) -> None:
        Start TCP server
    """
    __slots__ = [
        '__host',
        '__port',
        '__select',
        '__operations'
    ]

    CONNECTED_CLIENTS: Dict[str, socket.socket] = dict()
    TERMINATE = False

    def __init__(self) -> None:
        """TCP Socket server"""
        self.__host = ''
        self.__port = 0
        self.__operations: Dict[int, Any] = dict()
        self.__select = DefaultSelector()

    @property
    def HOST(self) -> str:
        """Returns socket host

        Returns
        -------
        str
            host
        """
        return self.__host

    @HOST.setter
    def HOST(self, host_name: str) -> None:
        """Set host name for TCP socket

        Parameters
        ----------
        host_name : str
            host name

        Raises
        ------
        ValueError
            if host name is not valid raises
        """
        if host_name not in ['', 'localhost']:
            try:
                ip_address(host_name)
                self.__host = host_name
            except ValueError:
                raise ValueError('IP address is Invalid')
        else:
            self.__host = host_name

    @property
    def PORT(self) -> int:
        """Returns TCP port

        Returns
        -------
        int
            TCP port
        """
        return self.__port

    @PORT.setter
    def PORT(self, port: int) -> None:
        """Set TCP socket port.

        Parameters
        ----------
        port : int
            TCP port

        Raises
        ------
        ValueError
            if port is not int raises
        """
        if not isinstance(port, int):
            raise ValueError('Port is Invalid')
        else:
            self.__port = port

    @property
    def operations(self) -> Dict[int, Any]:
        """Client requested operations

        Returns
        -------
        Dict[int, Any]
            registered operations
        """
        return self.__operations

    @operations.setter
    def operations(self, data: Tuple[int, Callable[[ArgT], ReturnT]]) -> None:
        """Registering new operations

        Parameters
        ----------
        data : Tuple[int, Callable[[ArgT], ReturnT]]
            new operations.
        """
        key, my_data = data
        operation = {key: my_data}
        self.__operations = {**operation, **self.__operations}

    def __cleanup_connections(self) -> None:
        """Cleaning up connections before terminating."""
        cls = self.__class__

        for conn in cls.CONNECTED_CLIENTS.values():
            print(f'Closing connection to: {conn.getpeername()}')
            self.__select.unregister(conn)
            conn.close()

    def __accept_wrapper(self, sock: socket.socket) -> None:
        """Accept new incoming connections.

        Parameters
        ----------
        sock : socket.socket
            socket instance.
        """
        cls = self.__class__

        _conn, addr = sock.accept()
        _conn.setblocking(False)
        print(f'Accepted incoming connection request from: {addr}')
        cls.CONNECTED_CLIENTS[str(addr[1])] = _conn

        _data = types.SimpleNamespace(addr=addr, inb=b'', outb=b'')
        _events = EVENT_READ | EVENT_WRITE

        self.__select.register(_conn, _events, data=_data)

    def __process_request(self, request: bytes, sock) -> None:
        """Mapping client request to registered process

        Parameters
        ----------
        request : bytes
            Client request
        sock : _type_
            socket instance
        """
        try:
            req = int(request)
            func = self.__operations.get(req)
            func(socket=sock)  # type: ignore

        except ValueError:
            print('Unknown request')

    def __service_connection(self, key: SelectorKey, mask: int) -> None:
        """Handle client messages.

        Parameters
        ----------
        key : SelectorKey
            Event
        mask : int
            Client mask
        """
        cls = self.__class__

        _sock: socket.socket = key.fileobj  # type: ignore
        data = key.data

        try:
            if mask & EVENT_READ:
                recv_data = _sock.recv(1)
                if recv_data:
                    print(recv_data)
                    Thread(target=self.__process_request,
                           args=(recv_data, _sock,), daemon=True).start()

        except ConnectionResetError:
            del cls.CONNECTED_CLIENTS[str(data.addr[1])]
            self.__select.unregister(_sock)
            _sock.close()

    def start_server(self) -> None:
        """Start TCP server"""
        cls = self.__class__

        try:
            tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            tcp_socket.bind((self.__host, self.__port))
            tcp_socket.listen()
            print(f'Starting TCP socket: {self.__host}:{self.__port}')
            tcp_socket.setblocking(False)

            self.__select.register(tcp_socket, EVENT_READ, data=None)

            while not cls.TERMINATE:
                _events = self.__select.select(timeout=None)
                for key, mask in _events:
                    if key.data is None:
                        self.__accept_wrapper(key.fileobj)  # type: ignore
                    else:
                        self.__service_connection(key, mask)

        except socket.error as e:
            if e.errno == errno.EADDRINUSE:
                print('Port is already in use.')

        except KeyboardInterrupt:
            _sep = f'\n{"*"*34}\n'
            print(f'\n{_sep}> Termination requested by user <{_sep}')

        finally:
            self.__cleanup_connections()
            self.__select.close()
            tcp_socket.close()

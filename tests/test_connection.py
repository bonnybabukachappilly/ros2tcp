import pytest
import socket

HOST = ''
PORT = 2030


@pytest.fixture
def socket_connection():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.connect((HOST, PORT))
        yield sock


def test_connection(socket_connection: socket.socket):
    socket_connection.sendall(b'1')
    print(socket_connection.recv(1024))

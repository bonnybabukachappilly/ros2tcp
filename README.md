# ros2TCP
Creating a bridge between ROS 2 and TCP Socket

## USAGE
Given example for using the library is given below.

### Creating TCP Socket
----

```python
# Import TCP Socket
from ros2tcp.TCPsocket import TCPSocketServer

# Create socket instance.
server = TCPSocketServer()

# Set 'HOST' name and 'PORT'
server.HOST = 'localhost'
server.PORT = 2040
```

### Creating ROS service client
----

```python
# Import required service type
from ros2tcp.plugins import register_callback, register_service, register_plugin
from std_srvs.srv import SetBool

# Create service request
request = SetBool.Request()
request.data = True

# Register service client
srv_1 = register_service(
    n_name='test_node_1',
    s_type=SetBool,
    s_name='test_service',
    req=request
)

srv_2 = register_service(
    n_name='test_node_2',
    s_type=SetBool,
    s_name='test_service',
    req=request
)

# Register callback when response received from the server
@register_callback
def my_callback_1(data):
    # 'data' argument is the response received from the ros service.
    # The data returned here will be send back to the socket client who requested this service.
    return data.message

@register_callback
def my_callback_2(_):
    # The data returned here will be send back to the socket client who requested this service.
    return 'done'

# Now map these service client to the plc request.
server.operations = register_plugin(
    srv=srv_1,
    cb=my_callback_1,
    key=1 # Whenever socket client sends '1', the 'srv_1' get called.
)

server.operations = register_plugin(
    srv=srv_2,
    cb=my_callback_1,
    key=1 # Whenever socket client sends '3', the 'srv_2' get called.
)

# NOTE: Currently key should be an integer.
```

### Running the Socket
----

```python
# import server runner
from ros2tcp.plugins import run_server

# Pass the created socket
run_server(socket=server)
```
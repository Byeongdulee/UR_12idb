import socket
import threading
import socketserver
import time

from urx.urscript import URScript
SOCKET_HOST = socket.getfqdn()
SOCKET_PORT = 50001
SOCKET_NAME = 'ursocket0'

class URsocket(URScript):
    def __init__(self, socket_host=SOCKET_HOST,
                 socket_port=SOCKET_PORT,
                 socket_name=SOCKET_NAME):
        self.socket_host = socket_host
        self.socket_port = socket_port
        self.socket_name = socket_name
        super(URsocket, self).__init__()

        # Reset connection to gripper
        self._socket_close(self.socket_name)
        self._socket_open(self.socket_host,
                          self.socket_port,
                          self.socket_name)
    def _send_var(self, var_name, value):
        self._socket_send_string("{}: {}".format(var_name, value), SOCKET_NAME)

# threaded server runs on a pc
class ThreadedTCPRequestHandler(socketserver.BaseRequestHandler):

    def handle(self):
        print("AAA")
        data = str(self.request.recv(1024), 'ascii')
        cur_thread = threading.current_thread()
        response = bytes("{}: {}".format(cur_thread.name, data), 'ascii')
        print(response)
        self.request.sendall(response)

class ThreadedTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    pass

# the ur controller is a client, sending a message through urscript socket commands:
class urclient(object):
    def __init__(self,
                 robot,
                 socket_host=SOCKET_HOST,
                 socket_port=SOCKET_PORT,
                 socket_name=SOCKET_NAME):
        self.robot = robot
        self.socket_host = socket_host
        self.socket_port = socket_port
        self.socket_name = socket_name

    def _get_new_script(self):
        urscript = URsocket(socket_host=self.socket_host,
                                 socket_port=self.socket_port, 
                                 socket_name=self.socket_name)
        return urscript
    
    def get_var(self, var_name='rq_pos', var_value=50):
        urscript = self._get_new_script()
        urscript._send_var(var_name, var_value)
    
        # Send the script
        self.robot.send_program(urscript())

        # sleep the code the same amount as the urscript to ensure that
        # the action completes
        time.sleep(0.1)        

def get_val(rob):
    HOST, PORT = "localhost", 10011
    server = ThreadedTCPServer((HOST, PORT), ThreadedTCPRequestHandler)
    with server:
        ip, port = server.server_address

        # Start a thread with the server -- that thread will then start one
        # more thread for each request
        print(ip, port)
        server_thread = threading.Thread(target=server.serve_forever)
        # Exit the server thread when the main thread terminates
        server_thread.daemon = True
        server_thread.start()
        #print("Server loop running in thread:", server_thread.name)
        time.sleep(1)
        urc = urclient(robot=rob.robot)
        urc.get_var()
        time.sleep(1)
        server.shutdown()

# this is an example client from https://docs.python.org/3/library/socketserver.html
def client(ip, port, message):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.connect((ip, port))
        sock.sendall(bytes(message, 'ascii'))
        response = str(sock.recv(1024), 'ascii')
        print("Received: {}".format(response))

if __name__ == "__main__":
    # Port 0 means to select an arbitrary unused port
    HOST, PORT = "localhost", 0

    server = ThreadedTCPServer((HOST, PORT), ThreadedTCPRequestHandler)
    with server:
        ip, port = server.server_address

        # Start a thread with the server -- that thread will then start one
        # more thread for each request
        server_thread = threading.Thread(target=server.serve_forever)
        # Exit the server thread when the main thread terminates
        server_thread.daemon = True
        server_thread.start()
        print("Server loop running in thread:", server_thread.name)

        client(ip, port, "Hello World 1")
        client(ip, port, "Hello World 2")
        client(ip, port, "Hello World 3")

        server.shutdown()
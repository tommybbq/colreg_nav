import asyncio
import threading
import websockets
from collections import deque
import json
import functools

ADDR = "localhost"
PORT = 9999
USERS = set()
RUDDER_ANGLE = 0
SAIL_ANGLE = 0

def rudder_event():
    return json.dumps({"type": "rudder", "data": RUDDER_ANGLE})


def sail_event():
    return json.dumps({"type": "sail", "data": SAIL_ANGLE})


"""
Handler. Takes a connection, recieves data and stores in thread shared read_queue and sends recieved data

"""
async def handler(websocket, read_queue):
    global USERS, RUDDER_ANGLE, SAIL_ANGLE  # idk what this does, was in example
    try:
        # Register user
        USERS.add(websocket)

        # Send current state to user
        await websocket.send(rudder_event())
        await websocket.send(sail_event())

        # Manage state changes
        async for message in websocket:
            event = json.loads(message)
            if event["type"] == "rudder":
                RUDDER_ANGLE = float(event["data"])
                read_queue.append( ("rudder", RUDDER_ANGLE) )

            if event["type"] == "sail":
                SAIL_ANGLE = float(event["data"])
                read_queue.append( ("sail" , SAIL_ANGLE) )


            await websocket.send(rudder_event())
            await websocket.send(sail_event())

    finally:
        # Unregister user
        USERS.remove(websocket)
        websockets.broadcast(USERS, rudder_event())
        websockets.broadcast(USERS, sail_event())


"""
Sychronous websocket server wrapper that uses async and threading to achieve this
Heavily borrowed from https://gist.github.com/dmfigol/3e7d5b84a16d076df02baa9f53271058
"""
class WebSocketServer():

    def __init__(self, addr="", port=9999, readbuffer=64):
        # stores the max capacity of the queues before items are discareded
        self._read_buffer = readbuffer

        # A limited queue that stores the data and start to delete data when full
        # Should be threadsafe, no locks needed
        self.read_queue = deque([],maxlen=self._read_buffer) 

        self.addr = addr
        self.port = port

        # setup thread
        self._loop = asyncio.new_event_loop()
        # need some way to pass queues into handler, use functool to create a "new" function with queues as default arguments
        # https://stackoverflow.com/questions/69765019/python-websockets-use-queue-to-pass-data-from-client-handler-to-a-coroutine-for
        handler_with_queue = functools.partial(handler, read_queue = self.read_queue)
        # server takes handler, addr, port, and event loop to operate on
        self._server = websockets.serve(handler_with_queue, self.addr, self.port, loop=self._loop)
        self._t = threading.Thread(target= self._start_background_loop, args=(self._loop, self._server), daemon=True)

    """
    Main coroutine of thread, starts server and asychronously loops it
    """
    def _start_background_loop(self, loop, server) -> None:
        loop.run_until_complete(server)
        loop.run_forever()

    """
    Starts the webserver. Return true on success, false if there is an exception.
    TODO: implement better error handling
    """
    def start(self) -> bool:
        print("Starting server")
        try:
            self._t.start()
            return True
        except Exception:
            print("Failed to start server")
            return False
        
    
    """
    Broadcast to server, use to send data
    """


    """
    Gets the NEWEST bit of data from the read queue (newest first)
    MAY NOT BE THREADSAFE? 
    """
    def read_latest(self) -> str:
        return self.read_queue.pop()
    """
    Pop data in chronological order (oldest first)
    Should use for most cases
    """
    def read_in_order(self) -> str:
        return self.read_queue.popleft()
    """
    Return true if the read queue is not empty
    """
    def read_avaliable(self):
        return len(self.read_queue) != 0
    """
    Clears the reading queue
    """
    def read_clear(self):
        self.read_queue.clear()


if __name__ == "__main__":
    import time

    ws = WebSocketServer(ADDR)
    ws.start()

    while True:
        # is something avaliable
        if ws.read_avaliable():
            data = ws.read_in_order()




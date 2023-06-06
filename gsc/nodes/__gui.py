#!/usr/bin/env python3
########################################################################
# Filename    : __gui.py
# Description : main app for gcs
# Author      : Man'O'AR
# modification: 17/01/2023
########################################################################

from time import  sleep
from threading import Thread,Lock

import rclpy

from ..utils.__utils_objects import DRONES_NAMES
from ..components.__windowInterface import DisplayWindow
from ..components.__rosThread import GCSNode

class App:

    def __init__(self, args=None ):

        self._gui = None
        self._node = None

        self._thread = None
        self._lock = Lock()

        self._is_running = True

        rclpy.init(args=args)
    

    def run(self):

        self._gui = DisplayWindow( Master=self, Names=DRONES_NAMES )

        self._thread = Thread(target=self.ros_thread)
        self._thread.daemon=True
        self._thread.start()
        
        self._gui._start(  )


    def _on_shutdown_selection( self, value ):
        
        if self._node is not None: 
            self._node._on_gui_closed(value)
            self._gui._stop()
            sleep(5)


    def _on_gui_drone_index_change( self, index ):
        self._node._remap_drone_control( index )

    def _on_frame_received(self, frame ):

        if self._gui is not None:
            
            with self._lock:

                self._gui._update_frame( frame )



    def ros_thread(self):

        self._node = GCSNode(Master=self) 

        try:

            rclpy.spin( self._node )

        except Exception as exception:
            print( "an exception has been raised while spinning the movement node : ", exception)

        except KeyboardInterrupt:
            print("user force interruption")
        
        
    def kill_ros(self):

        self._node._shutdown()
        self._node.destroy_node()

        rclpy.shutdown()


    def shutdown( self ):
        
        if self._node is not None:

            try:
                self.kill_ros() 
                
                if self._thread is not None: 
                    self._thread.join()
            
            except Exception:
                pass
                 
                 



def main(args=None):

    app = App(args)

    try :

        app.run()
        app.shutdown()

    except Exception as ex: 
        print( ex )



if __name__ == '__main__':
    main()
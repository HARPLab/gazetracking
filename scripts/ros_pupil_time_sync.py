#!/usr/bin/env python


# This script is modified from
# https://github.com/pupil-labs/pupil-helpers/blob/master/pupil_sync/pupil_time_sync_master.py


import rospy
from pyre import Pyre
from network_time_sync import Clock_Sync_Master

# import logging
# logger = logging.getLogger(__name__)
# logger.setLevel(logging.DEBUG)
# logging.getLogger('pyre').setLevel(logging.INFO)

# try:
#     from pyre import __version__
#     assert __version__ >= '0.3.1'
# except (ImportError, AssertionError):
#     raise Exception("Pyre version is too old. Please upgrade")

def run_time_sync_master(group):

    pts_group = group + '-time_sync-v1'
    print 'Looking for group {}'.format(pts_group)
  
    # the time source in the example is python time.time you can change this.
    # replace with an implementation that give your custom time in floating sec.
    clock_service = Clock_Sync_Master(rospy.get_time)

    # This example is a clock service only, not a clock follower.
    # Therefore the rank is designed to always trump all others.
    rank = 1000
    discovery = Pyre('pupil-helper-service')
    discovery.join(pts_group)
    discovery.start()
    print 'Joining "{}" group with rank {}'.format(pts_group, rank)

    def announce_clock_service_info():
        discovery.shout(pts_group, [repr(rank).encode(), repr(clock_service.port).encode()])

    try:
        for event in discovery.events():
            if event.type == 'JOIN' and event.group == pts_group:
                print '"{}" joined "{}" group. Announcing service.'.format(event.peer_name, pts_group)
                announce_clock_service_info()
    except KeyboardInterrupt:
        pass
    finally:
        print 'Leaving "{}" group'.format(pts_group)
        discovery.leave(pts_group)
        discovery.stop()
        clock_service.stop()


if __name__ == '__main__':
    rospy.init_node('pupil_clock_sync')
    run_time_sync_master('default')
    
    


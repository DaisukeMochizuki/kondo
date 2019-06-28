    import sys
    sys.path.append("kondo")
    from kondo import Kondo
    import time
    kondo = Kondo()
    kondo.run_motion(3)
    time.sleep(5.0)
    kondo.run_motion(4)
    kondo.run_motion(5)
    kondo.run_motion(6)
from crazyflie_py import Crazyswarm
import argparse
import time

def str2bool(flag):
    if isinstance(flag, bool):
        return flag
    
    if flag.lower() in ('yes', 'y', 'true', 't', '1'):
        return True
    
    if flag.lower() in ('no', 'n', 'false', 'f', '0'):
        return False
        
    raise argparse.ArgumentTypeError('Boolean value expected.')

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-l', '--log-flag',
        default=False,
        type=str2bool,
        help='True for enabling the logging, False for disabling it',
    )
    parser.add_argument(
        '-n', '--nn-flag',
        default=False,
        type=str2bool,
        help='True for enabling the neural network controller, False for disabling it',
    )
    parser.add_argument(
        '-d', '--delay',
        default=1.0,
        type=float,
        help='Delay in seconds between enabling the neural network controller and logging. WARNING: This must be different from 0.0',
    )
    args = parser.parse_args()

    swarm = Crazyswarm()
    cf = swarm.allcfs.crazyflies[0]

    states = {
        True: 1,
        False: 0
    }

    cf.setParam('nn_controller.activateNN', states[args.nn_flag])
    time.sleep(args.delay)
    cf.setParam('usd.logging', states[args.log_flag])

if __name__ == '__main__':
    main()
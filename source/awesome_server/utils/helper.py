import sys, getopt


def get_opt_path():
    server_path = None
    sm_path = None

    try:
        opts, args = getopt.getopt(sys.argv[1:], "hs:n:", ["sm=", "network="])
    except getopt.GetoptError:
        print '%s -s <path/to/sm_config> -n <path/to/network_config>' % sys.executable
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print '%s -s <path/to/sm_config> -n <path/to/network_config>' % sys.executable
            sys.exit()
        elif opt in ("-n", "--network"):
            server_path = arg
        elif opt in ("-s", "--sm"):
            sm_path = arg

    return sm_path, server_path

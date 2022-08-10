from bosdyn.client.estop import EstopEndpoint, EstopKeepAlive


class Estop:
    """
        parameter:
            - estop_client
            - estop_endpoint
            - estop_keepalive
        function:
            - start_estop
            - return_estop
    """
    def __init__(self, client):
        self.estop_client    = client
        self.estop_endpoint  = None
        self.estop_keepalive = None

    def toggle_estop(self):
        if self.estop_keepalive is None:
            return self.start_estop()
        else:
            return self.return_estop()

    def start_estop(self):
        self.estop_endpoint = EstopEndpoint(self.estop_client, 'Client', 9.0)
        self.estop_endpoint.force_simple_setup()
        self.estop_keepalive = EstopKeepAlive(self.estop_endpoint)
        return 'start estop'

    def return_estop(self):
        self.estop_keepalive.shutdown()
        self.estop_endpoint  = None
        self.estop_keepalive = None
        return 'return estop'

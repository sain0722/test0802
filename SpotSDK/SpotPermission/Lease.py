from bosdyn.client.lease import ResourceAlreadyClaimedError, LeaseKeepAlive


class Lease:
    """
        parameter:
            - lease
            - lease_client
            - lease_keepalive
        function:
            - start_lease
            - return_lease
    """
    def __init__(self, client):
        self.lease_client = client
        self.lease = None
        self.lease_keepalive = None

    def toggle_lease(self):
        if self.lease_keepalive is None:
            return self.start_lease()
        else:
            return self.return_lease()

    def start_lease(self):
        # check lease
        try:
            self.lease = self.lease_client.acquire()
        except ResourceAlreadyClaimedError as err:
            print("The robot's lease is currently in use. Check for a tablet connection or try again in a few seconds.")
            self.lease = self.lease_client.take()
            # return

        self.lease_keepalive = LeaseKeepAlive(self.lease_client)
        return 'start lease'

    def return_lease(self):
        """Shutdown lease keep-alive and return lease."""
        self.lease_keepalive.shutdown()
        self.lease_client.return_lease(self.lease)
        self.lease_keepalive = None
        self.lease = None

        return 'return lease'

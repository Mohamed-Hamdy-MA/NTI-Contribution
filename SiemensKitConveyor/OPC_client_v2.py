from opcua import Client

class opcuaClient:
    def __init__(self, server_url, node_ids):
        """Initialize the OPC UA client with server URL and a list of node IDs."""
        self.server_url = server_url
        self.node_ids = node_ids
        self.client = Client(server_url)
        self.nodes = {}

    def connect(self):
        """Connect to the OPC UA server and set up nodes."""
        
        try:
            self.client.connect()
            print("✅ Connected to OPC UA server")
            for node_id in self.node_ids:
                self.nodes[node_id] = self.client.get_node(node_id)
            return True
        
        except Exception as e:
            print(f"❌ Connection error: {e}")
            return False

    def disconnect(self):
        """Disconnect from the OPC UA server."""
        
        try:
            self.client.disconnect()
            print("🔌 Disconnected from server")
        
        except Exception as e:
            print(f"⚠️ Disconnection error: {e}")

    class SubHandler:
        """Subscription handler to process node value changes."""
        def datachange_notification(self, node, val, data):
            """Handle data change notifications."""
            print(f"🔄 Node {node} value changed to: {val}")

    def subscribe(self):
        """Subscribe to node changes and print updates."""

        # Create a subscription
        sub_handler = self.SubHandler()
        sub = self.client.create_subscription(1000, sub_handler)  # 1000ms interval
        for node_id in self.node_ids:
            try:
                sub.subscribe_data_change(self.nodes[node_id])
                print(f"✔ Subscribed to node {node_id}")
            except Exception as e:
                print(f"⚠️ Error subscribing to node {node_id}: {e}")


def main():

    # Configuration
    server_url = "opc.tcp://192.168.0.1:4840"
    node_ids = [
        "ns=3;s=\"SENSOR1\"",
        "ns=3;s=\"Barrier\"",
        "ns=3;s=\"SENSOR2\"",
        "ns=3;s=\"MOTOR RIGHT\"",
        "ns=3;s=\"MOTOR LEFT\"",
    ]

    # Create and run client
    client = opcuaClient(server_url, node_ids)
    try:
        if client.connect():
            client.subscribe()
            while True:
                # Keep the script running to listen for changes
                pass

    except KeyboardInterrupt:
        print("\n👋 Stopped by user")

    finally:
        client.disconnect()

if __name__ == "__main__":
    main()
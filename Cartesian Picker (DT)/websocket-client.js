// WebSocket Client for Python Integration
class WebSocketClient {
    constructor(onPositionUpdate) {
        this.ws = null;
        this.onPositionUpdate = onPositionUpdate;
        this.reconnectInterval = 3000; // 3 seconds
        this.reconnectTimer = null;
        this.statusBadge = document.getElementById('wsStatus');
        this.statusText = this.statusBadge.querySelector('.status-text');

        this.connect();
    }

    connect() {
        try {
            // Connect to WebSocket server on localhost:8765
            this.ws = new WebSocket('ws://localhost:8765');

            this.ws.onopen = () => {
                console.log('WebSocket connected');
                this.updateStatus(true);

                // Clear reconnect timer if exists
                if (this.reconnectTimer) {
                    clearTimeout(this.reconnectTimer);
                    this.reconnectTimer = null;
                }
            };

            this.ws.onmessage = (event) => {
                try {
                    const data = JSON.parse(event.data);

                    // Validate message format: {x: value, y: value, z: value}
                    if (this.isValidPositionData(data)) {
                        console.log('Received position data:', data);

                        // Call the position update callback
                        if (this.onPositionUpdate) {
                            this.onPositionUpdate(data);
                        }
                    } else {
                        console.warn('Invalid position data format:', data);
                    }
                } catch (error) {
                    console.error('Error parsing WebSocket message:', error);
                }
            };

            this.ws.onerror = (error) => {
                console.error('WebSocket error:', error);
            };

            this.ws.onclose = () => {
                console.log('WebSocket disconnected');
                this.updateStatus(false);

                // Attempt to reconnect
                this.scheduleReconnect();
            };

        } catch (error) {
            console.error('Failed to create WebSocket connection:', error);
            this.updateStatus(false);
            this.scheduleReconnect();
        }
    }

    scheduleReconnect() {
        if (!this.reconnectTimer) {
            console.log(`Reconnecting in ${this.reconnectInterval / 1000} seconds...`);
            this.reconnectTimer = setTimeout(() => {
                this.reconnectTimer = null;
                this.connect();
            }, this.reconnectInterval);
        }
    }

    isValidPositionData(data) {
        return data &&
            typeof data === 'object' &&
            'x' in data &&
            'y' in data &&
            'z' in data &&
            typeof data.x === 'number' &&
            typeof data.y === 'number' &&
            typeof data.z === 'number';
    }

    updateStatus(connected) {
        if (connected) {
            this.statusBadge.classList.add('connected');
            this.statusText.textContent = 'Connected';
        } else {
            this.statusBadge.classList.remove('connected');
            this.statusText.textContent = 'Disconnected';
        }
    }

    send(data) {
        if (this.ws && this.ws.readyState === WebSocket.OPEN) {
            this.ws.send(JSON.stringify(data));
        } else {
            console.warn('WebSocket is not connected');
        }
    }

    disconnect() {
        if (this.reconnectTimer) {
            clearTimeout(this.reconnectTimer);
            this.reconnectTimer = null;
        }

        if (this.ws) {
            this.ws.close();
            this.ws = null;
        }
    }
}

// Export for use in other scripts
window.WebSocketClient = WebSocketClient;

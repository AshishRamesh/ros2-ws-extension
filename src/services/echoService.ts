import { spawn, ChildProcess } from 'child_process';

export interface EchoMessage {
    data: string;
    timestamp: Date;
}

export class EchoService {
    private currentProcess: ChildProcess | null = null;
    private messageCount: number = 0;
    private lastTimestamp: Date | null = null;
    private timestamps: Date[] = [];

    /**
     * Starts echoing a ROS 2 topic
     */
    startEcho(topic: string, callback: (message: EchoMessage) => void): void {
        // Stop any existing echo
        this.stopEcho();

        // Reset counters
        this.messageCount = 0;
        this.lastTimestamp = null;
        this.timestamps = [];

        // Start new echo process
        this.currentProcess = spawn('ros2', ['topic', 'echo', topic]);

        let buffer = '';

        this.currentProcess.stdout?.on('data', (data: Buffer) => {
            buffer += data.toString();

            // Split by '---' which separates messages in ros2 topic echo
            const messages = buffer.split('---');

            // Keep the last incomplete message in the buffer
            buffer = messages.pop() || '';

            // Process complete messages
            for (const msg of messages) {
                if (msg.trim()) {
                    const timestamp = new Date();
                    this.messageCount++;
                    this.lastTimestamp = timestamp;
                    this.timestamps.push(timestamp);

                    // Keep only last 10 timestamps for frequency calculation
                    if (this.timestamps.length > 10) {
                        this.timestamps.shift();
                    }

                    callback({
                        data: msg.trim(),
                        timestamp
                    });
                }
            }
        });

        this.currentProcess.stderr?.on('data', (data: Buffer) => {
            console.error(`Echo error: ${data.toString()}`);
        });

        this.currentProcess.on('close', (code) => {
            console.log(`Echo process exited with code ${code}`);
        });
    }

    /**
     * Stops the current echo process
     */
    stopEcho(): void {
        if (this.currentProcess) {
            this.currentProcess.kill();
            this.currentProcess = null;
        }
    }

    /**
     * Gets the current message count
     */
    getMessageCount(): number {
        return this.messageCount;
    }

    /**
     * Gets the last message timestamp
     */
    getLastTimestamp(): Date | null {
        return this.lastTimestamp;
    }

    /**
     * Calculates approximate frequency in Hz
     */
    getFrequency(): number {
        if (this.timestamps.length < 2) {
            return 0;
        }

        const first = this.timestamps[0].getTime();
        const last = this.timestamps[this.timestamps.length - 1].getTime();
        const duration = (last - first) / 1000; // seconds

        if (duration === 0) {
            return 0;
        }

        return (this.timestamps.length - 1) / duration;
    }

    /**
     * Checks if echo is currently running
     */
    isRunning(): boolean {
        return this.currentProcess !== null;
    }
}

import { exec } from 'child_process';
import { promisify } from 'util';

const execAsync = promisify(exec);

export class TopicsService {
    /**
     * Lists all available ROS 2 topics
     */
    async listTopics(): Promise<string[]> {
        try {
            const { stdout } = await execAsync('ros2 topic list');
            return stdout
                .trim()
                .split('\n')
                .filter(topic => topic.length > 0);
        } catch (error) {
            console.error('Failed to list topics:', error);
            return [];
        }
    }

    /**
     * Gets the message type for a specific topic
     */
    async getTopicType(topic: string): Promise<string> {
        try {
            const { stdout } = await execAsync(`ros2 topic type ${topic}`);
            return stdout.trim();
        } catch (error) {
            console.error(`Failed to get type for topic ${topic}:`, error);
            return 'Unknown';
        }
    }

    /**
     * Gets topic info including type and publisher count
     */
    async getTopicInfo(topic: string): Promise<{ type: string; publishers: number; subscribers: number }> {
        try {
            const { stdout } = await execAsync(`ros2 topic info ${topic}`);
            const lines = stdout.split('\n');

            let type = 'Unknown';
            let publishers = 0;
            let subscribers = 0;

            for (const line of lines) {
                if (line.includes('Type:')) {
                    type = line.split('Type:')[1].trim();
                } else if (line.includes('Publisher count:')) {
                    publishers = parseInt(line.split('Publisher count:')[1].trim()) || 0;
                } else if (line.includes('Subscription count:')) {
                    subscribers = parseInt(line.split('Subscription count:')[1].trim()) || 0;
                }
            }

            return { type, publishers, subscribers };
        } catch (error) {
            console.error(`Failed to get info for topic ${topic}:`, error);
            return { type: 'Unknown', publishers: 0, subscribers: 0 };
        }
    }
}

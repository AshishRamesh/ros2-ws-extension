import * as vscode from 'vscode';
import { TopicsService } from '../services/topicsService';

export class BagRecordingWizard {
    constructor(private topicsService: TopicsService) { }

    async run(): Promise<void> {
        // Step 1: Get bag file name
        const bagName = await vscode.window.showInputBox({
            prompt: 'Enter bag file name (without extension)',
            placeHolder: 'my_recording',
            validateInput: (value) => {
                if (!value || value.trim().length === 0) {
                    return 'Bag name cannot be empty';
                }
                if (!/^[a-zA-Z0-9_-]+$/.test(value)) {
                    return 'Bag name can only contain letters, numbers, hyphens, and underscores';
                }
                return null;
            }
        });

        if (!bagName) {
            return; // User cancelled
        }

        // Step 2: Select output directory
        const uris = await vscode.window.showOpenDialog({
            canSelectFiles: false,
            canSelectFolders: true,
            canSelectMany: false,
            openLabel: 'Select Output Directory'
        });

        if (!uris || uris.length === 0) {
            return; // User cancelled
        }

        const outputDir = uris[0].fsPath;

        // Step 3: Get available topics
        const topics = await this.topicsService.listTopics();

        if (topics.length === 0) {
            vscode.window.showWarningMessage('No ROS 2 topics available');
            return;
        }

        // Step 4: Topic selection
        const topicSelection = await vscode.window.showQuickPick(
            [
                { label: 'All Topics', description: 'Record all available topics', value: 'all' },
                { label: 'Select Specific Topics', description: 'Choose individual topics to record', value: 'select' }
            ],
            { placeHolder: 'Choose topic selection method' }
        );

        if (!topicSelection) {
            return; // User cancelled
        }

        let selectedTopics: string[] = [];

        if (topicSelection.value === 'all') {
            selectedTopics = topics;
        } else {
            // Show multi-select for topics
            const topicItems = topics.map(topic => ({ label: topic, picked: false }));
            const selected = await vscode.window.showQuickPick(topicItems, {
                canPickMany: true,
                placeHolder: 'Select topics to record (use Space to select, Enter to confirm)'
            });

            if (!selected || selected.length === 0) {
                vscode.window.showWarningMessage('No topics selected');
                return;
            }

            selectedTopics = selected.map(item => item.label);
        }

        // Step 5: Start recording
        await this.startRecording(bagName, outputDir, selectedTopics);
    }

    private async startRecording(bagName: string, outputDir: string, topics: string[]): Promise<void> {
        const terminal = vscode.window.createTerminal(`ROS 2 Bag: ${bagName}`);
        terminal.show();

        // Build ros2 bag record command
        const topicsArg = topics.join(' ');
        const command = `cd "${outputDir}" && ros2 bag record -o ${bagName} ${topicsArg}`;

        terminal.sendText(command);

        vscode.window.showInformationMessage(
            `Recording ${topics.length} topic(s) to ${bagName}. Press Ctrl+C in the terminal to stop.`,
            'Open Folder'
        ).then(selection => {
            if (selection === 'Open Folder') {
                vscode.commands.executeCommand('revealFileInOS', vscode.Uri.file(outputDir));
            }
        });
    }
}

import * as vscode from 'vscode';

export class BagPlaybackWizard {
    async run(): Promise<void> {
        // Step 1: Select bag file
        const uris = await vscode.window.showOpenDialog({
            canSelectFiles: false,
            canSelectFolders: true,
            canSelectMany: false,
            openLabel: 'Select Bag Directory',
            title: 'Select ROS 2 Bag Directory'
        });

        if (!uris || uris.length === 0) {
            return; // User cancelled
        }

        const bagPath = uris[0].fsPath;

        // Step 2: Playback rate
        const rateInput = await vscode.window.showInputBox({
            prompt: 'Enter playback rate (default: 1.0)',
            placeHolder: '1.0',
            value: '1.0',
            validateInput: (value) => {
                const rate = parseFloat(value);
                if (isNaN(rate) || rate <= 0) {
                    return 'Rate must be a positive number';
                }
                return null;
            }
        });

        if (rateInput === undefined) {
            return; // User cancelled
        }

        const rate = parseFloat(rateInput);

        // Step 3: Loop option
        const loopOption = await vscode.window.showQuickPick(
            [
                { label: 'Play Once', description: 'Play bag file once and stop', value: false },
                { label: 'Loop Playback', description: 'Continuously loop the bag file', value: true }
            ],
            { placeHolder: 'Choose playback mode' }
        );

        if (!loopOption) {
            return; // User cancelled
        }

        // Step 4: Additional options
        const additionalOptions = await vscode.window.showQuickPick(
            [
                { label: 'Start Paused', description: 'Start in paused state (press Space to play)', picked: false },
                { label: 'Clock', description: 'Publish simulated clock time', picked: false },
                { label: 'Disable Keyboard Controls', description: 'Disable interactive keyboard controls', picked: false }
            ],
            {
                canPickMany: true,
                placeHolder: 'Select additional options (optional, press Enter to skip)'
            }
        );

        // Build command
        await this.startPlayback(bagPath, rate, loopOption.value, additionalOptions || []);
    }

    private async startPlayback(
        bagPath: string,
        rate: number,
        loop: boolean,
        options: Array<{ label: string }>
    ): Promise<void> {
        const terminal = vscode.window.createTerminal(`ROS 2 Bag Play`);
        terminal.show();

        // Build ros2 bag play command
        let command = `ros2 bag play "${bagPath}"`;

        // Add rate
        if (rate !== 1.0) {
            command += ` --rate ${rate}`;
        }

        // Add loop
        if (loop) {
            command += ` --loop`;
        }

        // Add additional options
        const optionLabels = options.map(o => o.label);
        if (optionLabels.includes('Start Paused')) {
            command += ` --start-paused`;
        }
        if (optionLabels.includes('Clock')) {
            command += ` --clock`;
        }
        if (optionLabels.includes('Disable Keyboard Controls')) {
            command += ` --disable-keyboard-controls`;
        }

        terminal.sendText(command);

        const loopText = loop ? ' (looping)' : '';
        const rateText = rate !== 1.0 ? ` at ${rate}x speed` : '';
        vscode.window.showInformationMessage(
            `Playing bag${rateText}${loopText}. Use terminal controls to pause/resume.`
        );
    }
}

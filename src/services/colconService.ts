import * as vscode from 'vscode';
import * as fs from 'fs';
import * as path from 'path';
import { runCommand } from './commandRunner';

export class ColconService {
    private onBuildCompleteCallback?: () => void;

    constructor(private outputChannel: vscode.OutputChannel) { }

    public setOnBuildComplete(callback: () => void): void {
        this.onBuildCompleteCallback = callback;
    }

    /**
     * Initialize a ROS 2 workspace
     */
    async initWorkspace(workspacePath: string): Promise<void> {
        this.outputChannel.show();
        this.outputChannel.appendLine('Initializing ROS 2 workspace...');

        const srcPath = path.join(workspacePath, 'src');

        // Create src directory if it doesn't exist
        if (!fs.existsSync(srcPath)) {
            fs.mkdirSync(srcPath, { recursive: true });
            this.outputChannel.appendLine(`Created directory: ${srcPath}`);
        } else {
            this.outputChannel.appendLine(`Directory already exists: ${srcPath}`);
        }

        this.outputChannel.appendLine('Workspace initialized successfully!');
    }

    /**
     * Build the workspace using colcon
     */
    async build(workspacePath: string, options?: BuildOptions): Promise<void> {
        this.outputChannel.show();
        this.outputChannel.appendLine('Starting colcon build...');

        // Validate workspace
        const srcPath = path.join(workspacePath, 'src');
        if (!fs.existsSync(srcPath)) {
            throw new Error('Invalid ROS 2 workspace: src/ directory not found');
        }

        // Build command
        let cmd = 'colcon build';

        if (options?.symlinkInstall) {
            cmd += ' --symlink-install';
        }

        if (options?.packagesSelect && options.packagesSelect.length > 0) {
            cmd += ` --packages-select ${options.packagesSelect.join(' ')}`;
        }

        try {
            await runCommand(cmd, workspacePath, this.outputChannel);
            this.outputChannel.appendLine('Build completed successfully!');
            vscode.window.showInformationMessage('Build completed successfully!');

            // Notify listeners that build is complete
            if (this.onBuildCompleteCallback) {
                this.onBuildCompleteCallback();
            }
        } catch (error) {
            this.outputChannel.appendLine(`\n✗ Build failed: ${error}`);
            throw error;
        }
    }

    /**
     * Clean build artifacts
     */
    async clean(workspacePath: string): Promise<void> {
        this.outputChannel.show();
        this.outputChannel.appendLine('Cleaning build artifacts...');

        const dirsToRemove = ['build', 'install', 'log'];

        for (const dir of dirsToRemove) {
            const dirPath = path.join(workspacePath, dir);
            if (fs.existsSync(dirPath)) {
                this.outputChannel.appendLine(`Removing: ${dirPath}`);
                fs.rmSync(dirPath, { recursive: true, force: true });
            }
        }

        this.outputChannel.appendLine('✓ Clean completed successfully!');
    }

    /**
     * Generate .gitignore file with standard ROS 2 patterns
     */
    async generateGitignore(workspacePath: string): Promise<void> {
        const gitignorePath = path.join(workspacePath, '.gitignore');

        if (fs.existsSync(gitignorePath)) {
            return;
        }

        this.outputChannel.appendLine('Generating .gitignore file...');

        const content = `# ROS 2 build artifacts
build/
install/
log/

# Python
__pycache__/
*.py[cod]
*$py.class

# C++
cmake-build-debug/
.vscode/
*.swp
*~

# Colcon
COLCON_IGNORE
`;

        try {
            fs.writeFileSync(gitignorePath, content);
            this.outputChannel.appendLine('✓ Created .gitignore file');
        } catch (error) {
            this.outputChannel.appendLine(`⚠ Failed to create .gitignore: ${error}`);
        }
    }
}

export interface BuildOptions {
    symlinkInstall?: boolean;
    packagesSelect?: string[];
}

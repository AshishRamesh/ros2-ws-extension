import { spawn } from 'child_process';
import * as vscode from 'vscode';

/**
 * Runs a shell command and returns a promise
 * @param cmd Command to execute
 * @param cwd Working directory
 * @param outputChannel Optional output channel for logging
 * @returns Promise that resolves when command completes
 */
export function runCommand(
    cmd: string,
    cwd: string,
    outputChannel?: vscode.OutputChannel
): Promise<void> {
    return new Promise((resolve, reject) => {
        if (outputChannel) {
            outputChannel.appendLine(`\n$ ${cmd}`);
            outputChannel.appendLine(`Working directory: ${cwd}`);
            outputChannel.appendLine('---');
        }

        const child = spawn(cmd, {
            cwd,
            shell: true,
            stdio: ['ignore', 'pipe', 'pipe']
        });

        let stdout = '';
        let stderr = '';

        child.stdout?.on('data', (data) => {
            const text = data.toString();
            stdout += text;
            if (outputChannel) {
                outputChannel.append(text);
            }
        });

        child.stderr?.on('data', (data) => {
            const text = data.toString();
            stderr += text;
            if (outputChannel) {
                outputChannel.append(text);
            }
        });

        child.on('error', (error) => {
            if (outputChannel) {
                outputChannel.appendLine(`\nError: ${error.message}`);
            }
            reject(error);
        });

        child.on('close', (code) => {
            if (outputChannel) {
                outputChannel.appendLine(`\nProcess exited with code ${code}`);
            }

            if (code === 0) {
                resolve();
            } else {
                reject(new Error(`Command failed with exit code ${code}`));
            }
        });
    });
}

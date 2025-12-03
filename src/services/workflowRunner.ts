import * as vscode from 'vscode';
import { Workflow, WorkflowStep } from '../types/workflow';
import { LaunchService } from '../services/launchService';
import { NodeDiscoveryService } from '../services/nodeDiscoveryService';
import * as fs from 'fs';
import * as path from 'path';

export class WorkflowRunner {
    private activeTerminals: vscode.Terminal[] = [];
    private isRunning = false;
    private runningWorkflowName?: string;
    private sourceCommand: string = '';
    private lastTerminal: vscode.Terminal | undefined;

    private _onDidStateChange = new vscode.EventEmitter<void>();
    public readonly onDidStateChange = this._onDidStateChange.event;

    constructor(
        private launchService: LaunchService,
        private nodeDiscoveryService: NodeDiscoveryService
    ) { }

    public getRunningWorkflowName(): string | undefined {
        return this.runningWorkflowName;
    }

    public async runWorkflow(workflow: Workflow): Promise<void> {
        if (this.isRunning) {
            vscode.window.showWarningMessage('A workflow is already running');
            return;
        }

        const workspaceRoot = vscode.workspace.workspaceFolders?.[0]?.uri.fsPath;
        if (!workspaceRoot) {
            vscode.window.showErrorMessage('No workspace folder open');
            return;
        }

        // Validate workflow
        const validation = await this.validateWorkflow(workflow, workspaceRoot);
        if (!validation.valid) {
            const proceed = await vscode.window.showWarningMessage(
                `Workflow validation warnings:\n${validation.warnings.join('\n')}\n\nContinue anyway?`,
                'Yes', 'No'
            );
            if (proceed !== 'Yes') {
                return;
            }
        }


        this.isRunning = true;
        this.runningWorkflowName = workflow.name;
        this.activeTerminals = [];
        this.lastTerminal = undefined;
        this._onDidStateChange.fire();

        // Default source command
        const setupBash = path.join(workspaceRoot, 'install', 'setup.bash');
        if (fs.existsSync(setupBash)) {
            this.sourceCommand = `source ${setupBash}`;
        }

        try {
            await this.executeWorkflow(workflow, workspaceRoot);
            vscode.window.showInformationMessage(`Workflow "${workflow.name}" started successfully`);
        } catch (error) {
            vscode.window.showErrorMessage(`Workflow failed: ${error}`);
            this.stopWorkflow();
        } finally {
            this.isRunning = false;
            this.runningWorkflowName = undefined;
            this._onDidStateChange.fire();
        }
    }

    private async validateWorkflow(workflow: Workflow, workspaceRoot: string): Promise<{ valid: boolean, warnings: string[] }> {
        const warnings: string[] = [];

        // Check if workspace is built
        const installPath = path.join(workspaceRoot, 'install');
        if (!fs.existsSync(installPath)) {
            warnings.push('Workspace is not built (install/ directory not found)');
        }

        // Validate each step
        for (const step of workflow.steps) {
            if (!step.enabled) { continue; }

            switch (step.type) {
                case 'launch':
                    if (!step.config.package || !step.config.launchFile) {
                        warnings.push(`Launch step missing package or file: ${step.config.description || 'unnamed'}`);
                    }
                    break;

                case 'run':
                    if (!step.config.package || !step.config.nodeName) {
                        warnings.push(`Run step missing package or node: ${step.config.description || 'unnamed'}`);
                    }
                    break;

                case 'command':
                    if (!step.config.command) {
                        warnings.push(`Command step missing command: ${step.config.description || 'unnamed'}`);
                    }
                    break;
            }
        }

        return {
            valid: warnings.length === 0,
            warnings
        };
    }

    private async executeWorkflow(workflow: Workflow, workspaceRoot: string): Promise<void> {
        // Execute steps sequentially
        for (let i = 0; i < workflow.steps.length; i++) {
            if (!this.isRunning) { break; }

            const step = workflow.steps[i];





            await this.executeStep(step, workspaceRoot);
        }
    }

    private async executeStep(step: WorkflowStep, workspaceRoot: string): Promise<void> {
        let terminal: vscode.Terminal | undefined;
        let command = '';
        let terminalName = '';

        switch (step.type) {
            case 'source':
                // Update the source command for future steps
                // If it's a custom source command, we might want to support that, but usually it's just the workspace
                // For now, we assume 'source' step means "ensure workspace is sourced" which we do by default,
                // but if we wanted to support custom setup files, we'd parse it here.
                // Since the config doesn't have a path for 'source' type (it's just a flag in the UI currently),
                // we'll stick to the default behavior or maybe re-source.
                // Actually, let's assume if they added a source step, they want to make sure it's sourced.
                // We already set this.sourceCommand in runWorkflow.
                // If we want to support custom paths, we'd need to update the step config.
                // For now, let's just log it.

                break;

            case 'launch':
                terminalName = `ROS 2: ${step.config.package} ${step.config.launchFile}`;
                command = `ros2 launch ${step.config.package} ${step.config.launchFile}`;
                break;

            case 'run':
                terminalName = `ROS 2: ${step.config.package} ${step.config.nodeName}`;
                command = `ros2 run ${step.config.package} ${step.config.nodeName}`;
                break;

            case 'command':
                const cmdPreview = (step.config.command || 'cmd').substring(0, 20);
                terminalName = `ROS 2: Command ${cmdPreview}...`;
                command = step.config.command || '';
                break;

            case 'delay':
                const delayMs = step.config.delayMs || 1000;
                await this.delay(delayMs);
                return; // No terminal needed for delay
        }

        if (command) {
            // Check if we should reuse the last terminal
            if (step.sameTerminal && this.lastTerminal) {
                terminal = this.lastTerminal;
            } else {
                // Create or get specific terminal
                terminal = this.getOrCreateTerminal(terminalName);
                this.lastTerminal = terminal;

                // Only add to active terminals if not already tracked
                if (!this.activeTerminals.includes(terminal)) {
                    this.activeTerminals.push(terminal);
                }

                // Source environment for new terminals (or when switching terminals)
                if (this.sourceCommand) {
                    terminal.sendText(this.sourceCommand);
                }
            }

            terminal.show();
            terminal.sendText(command);

            // Small delay to ensure terminal is ready and command is sent
            await this.delay(500);
        }
    }

    private getOrCreateTerminal(name: string): vscode.Terminal {
        // Check if terminal exists
        const existing = vscode.window.terminals.find(t => t.name === name);
        if (existing) {
            return existing;
        }
        return vscode.window.createTerminal(name);
    }

    private delay(ms: number): Promise<void> {
        return new Promise(resolve => setTimeout(resolve, ms));
    }

    public stopWorkflow(): void {
        this.isRunning = false;
        this.runningWorkflowName = undefined;
        this._onDidStateChange.fire();

        for (const terminal of this.activeTerminals) {
            try {
                terminal.sendText('\x03'); // Send Ctrl+C
            } catch (e) {
                console.error('Error stopping terminal:', e);
            }
        }
        // Clear the list after stopping
        this.activeTerminals = [];
    }
}

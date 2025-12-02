import * as vscode from 'vscode';
import { WorkflowConfiguration } from '../types/workflow';
import { WorkflowRunner } from '../services/workflowRunner';

export class WorkflowViewProvider implements vscode.TreeDataProvider<WorkflowTreeItem> {
    private _onDidChangeTreeData: vscode.EventEmitter<WorkflowTreeItem | undefined | null | void> = new vscode.EventEmitter<WorkflowTreeItem | undefined | null | void>();
    readonly onDidChangeTreeData: vscode.Event<WorkflowTreeItem | undefined | null | void> = this._onDidChangeTreeData.event;

    constructor(private workflowRunner: WorkflowRunner) {
        // Listen for configuration changes to refresh the view
        vscode.workspace.onDidChangeConfiguration(e => {
            if (e.affectsConfiguration('ros2Toolkit.workflowSteps')) {
                this.refresh();
            }
        });

        // Listen for runner state changes
        this.workflowRunner.onDidStateChange(() => this.refresh());
    }

    refresh(): void {
        this._onDidChangeTreeData.fire();
    }

    getTreeItem(element: WorkflowTreeItem): vscode.TreeItem {
        return element;
    }

    getChildren(element?: WorkflowTreeItem): Thenable<WorkflowTreeItem[]> {
        if (element) {
            return Promise.resolve([]);
        }

        const config = vscode.workspace.getConfiguration('ros2Toolkit');
        const workflowConfig: WorkflowConfiguration = config.get('workflowSteps') || { workflows: [] };

        if (workflowConfig.workflows.length === 0) {
            const item = new WorkflowTreeItem(
                'No workflows configured',
                vscode.TreeItemCollapsibleState.None,
                new vscode.ThemeIcon('info')
            );
            item.description = 'Click "+" to create one';
            item.command = {
                command: 'ros2.configureWorkflow',
                title: 'Configure Workflow'
            };
            return Promise.resolve([item]);
        }

        const runningWorkflow = this.workflowRunner.getRunningWorkflowName();

        const items = workflowConfig.workflows.map(workflow => {
            const isRunning = workflow.name === runningWorkflow;

            // Determine icon: Stop (square) if running, Play (triangle) if not
            const icon = isRunning ? new vscode.ThemeIcon('debug-stop') : new vscode.ThemeIcon('play');

            const item = new WorkflowTreeItem(
                workflow.name,
                vscode.TreeItemCollapsibleState.None,
                icon
            );

            item.description = isRunning ? 'Running...' : `${workflow.steps.length} steps`;
            item.contextValue = 'workflow';

            // Toggle behavior: Click to Run, Click again to Stop
            if (isRunning) {
                item.command = {
                    command: 'ros2.stopWorkflow',
                    title: 'Stop Workflow'
                };
            } else {
                item.command = {
                    command: 'ros2.runWorkflow',
                    title: 'Run Workflow',
                    arguments: [workflow] // Pass workflow object
                };
            }

            return item;
        });

        return Promise.resolve(items);
    }
}

class WorkflowTreeItem extends vscode.TreeItem {
    constructor(
        public readonly label: string,
        public readonly collapsibleState: vscode.TreeItemCollapsibleState,
        public readonly iconPath?: vscode.ThemeIcon
    ) {
        super(label, collapsibleState);
        this.iconPath = iconPath;
    }
}

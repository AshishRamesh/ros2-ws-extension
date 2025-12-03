export interface WorkflowStep {
    id: string;
    type: 'source' | 'launch' | 'run' | 'command' | 'delay';
    enabled: boolean;
    sameTerminal?: boolean;
    config: {
        // For 'launch' type
        package?: string;
        launchFile?: string;

        // For 'run' type
        nodeName?: string;

        // For 'command' type
        command?: string;

        // For 'delay' type
        delayMs?: number;

        // Common
        description?: string;
    };
}

export interface Workflow {
    name: string;
    steps: WorkflowStep[];
}

export interface WorkflowConfiguration {
    workflows: Workflow[];
    activeWorkflow?: string;
}

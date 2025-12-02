export interface WorkflowStep {
    id: string;
    type: 'source' | 'launch' | 'run' | 'script' | 'delay';
    enabled: boolean;
    config: {
        // For 'launch' type
        package?: string;
        launchFile?: string;

        // For 'run' type
        nodeName?: string;

        // For 'script' type
        scriptPath?: string;

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

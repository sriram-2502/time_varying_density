function [y_backward] = backwardEuler(agent_id, y, deltaT)
    % Persistent variable to store agent states
    persistent agentStates
    
    % Initialize the map if it hasn't been initialized yet
    if isempty(agentStates)
        agentStates = containers.Map('KeyType', 'double', 'ValueType', 'any');
    end

    % Check if the agent's state exists in the map
    if ~isKey(agentStates, agent_id)
        disp(['Setting zero for agent ' num2str(agent_id) ' in backward euler'])
        agentStates(agent_id) = 0; % Initialize y_prev for the agent
    end

    % Retrieve the previous state for the agent
    y_prev = agentStates(agent_id);

    % Calculate the backward Euler approximation
    y_backward = (y - y_prev) / deltaT;
    
    % Update the agent's previous state
    agentStates(agent_id) = y;
end

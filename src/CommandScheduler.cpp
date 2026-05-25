#include "CommandScheduler.h"
#include <algorithm>

void CommandScheduler::schedule(std::shared_ptr<Command> command) {
    if (!command) return;

    // Avoid scheduling the same command instance twice
    auto it = std::find_if(m_activeCommands.begin(), m_activeCommands.end(),
        [&command](const ScheduledCommand& sc) {
            return sc.command == command;
        });

    if (it == m_activeCommands.end()) {
        m_activeCommands.push_back({command, false});
    }
}

void CommandScheduler::run(double dt) {
    // We use a manual index loop or copy-then-iterate because execute() or end()
    // of a command could modify the command scheduler list (e.g. scheduling a new one).
    // A standard robust way is to iterate over a copy of the pointers or handle iteration carefully.
    std::vector<ScheduledCommand> commandsToRun = m_activeCommands;
    std::vector<std::shared_ptr<Command>> finishedCommands;

    for (auto& sc : commandsToRun) {
        // Find the actual element in our active list to check if it wasn't cancelled mid-run
        auto activeIt = std::find_if(m_activeCommands.begin(), m_activeCommands.end(),
            [&sc](const ScheduledCommand& item) { return item.command == sc.command; });
        
        if (activeIt == m_activeCommands.end()) {
            continue; // Command was cancelled/removed during this tick
        }

        if (!activeIt->initialized) {
            activeIt->command->initialize();
            activeIt->initialized = true;
        }

        activeIt->command->execute(dt);

        if (activeIt->command->isFinished()) {
            activeIt->command->end();
            finishedCommands.push_back(activeIt->command);
        }
    }

    // Remove finished commands
    for (auto& finished : finishedCommands) {
        m_activeCommands.erase(
            std::remove_if(m_activeCommands.begin(), m_activeCommands.end(),
                [&finished](const ScheduledCommand& sc) {
                    return sc.command == finished;
                }),
            m_activeCommands.end()
        );
    }
}

void CommandScheduler::cancel(std::shared_ptr<Command> command) {
    if (!command) return;

    auto it = std::find_if(m_activeCommands.begin(), m_activeCommands.end(),
        [&command](const ScheduledCommand& sc) {
            return sc.command == command;
        });

    if (it != m_activeCommands.end()) {
        if (it->initialized) {
            it->command->end();
        }
        m_activeCommands.erase(it);
    }
}

void CommandScheduler::cancelAll() {
    for (auto& sc : m_activeCommands) {
        if (sc.initialized) {
            sc.command->end();
        }
    }
    m_activeCommands.clear();
}

bool CommandScheduler::hasActiveCommands() const {
    return !m_activeCommands.empty();
}

std::vector<std::shared_ptr<Command>> CommandScheduler::getActiveCommands() const {
    std::vector<std::shared_ptr<Command>> list;
    for (const auto& sc : m_activeCommands) {
        list.push_back(sc.command);
    }
    return list;
}

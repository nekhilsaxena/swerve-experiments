#pragma once

#include "Command.h"
#include <vector>
#include <memory>

class CommandScheduler {
public:
    static CommandScheduler& getInstance() {
        static CommandScheduler instance;
        return instance;
    }

    // Scheduling and control
    void schedule(std::shared_ptr<Command> command);
    void run(double dt);
    void cancel(std::shared_ptr<Command> command);
    void cancelAll();

    // State queries
    bool hasActiveCommands() const;
    std::vector<std::shared_ptr<Command>> getActiveCommands() const;

private:
    CommandScheduler() = default;
    ~CommandScheduler() = default;
    CommandScheduler(const CommandScheduler&) = delete;
    CommandScheduler& operator=(const CommandScheduler&) = delete;

    struct ScheduledCommand {
        std::shared_ptr<Command> command;
        bool initialized{false};
    };

    std::vector<ScheduledCommand> m_activeCommands;
};

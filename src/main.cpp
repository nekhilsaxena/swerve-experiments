#define OLC_PGE_APPLICATION
#include "olcPixelGameEngine.h"
#define _USE_MATH_DEFINES
#include <deque>
#include <cmath>
#include <vector>
#include <string>
#include <iomanip>
#include <optional>
#include <iostream>

#include "Physics.h"
#include "SwerveModule.h"
#include "SwerveDrive.h"
#include "DriveRequest.h"
#include "SwerveDriveUtil.h"
#include "AutonomousRoutines.h"
#include "Constants.h"
#include "CommandScheduler.h"
#include "DrivetrainCommands.h"

using namespace Physics;

// Dashboard Button Helper
struct Button
{
    int x;
    int y;
    int w;
    int h;
    std::string label;
    olc::Pixel normalColor{60, 60, 60};
    olc::Pixel hoverColor{90, 90, 90};
    olc::Pixel activeColor{130, 130, 130};

    bool isHovered(int mx, int my) const
    {
        return mx >= x && mx < x + w && my >= y && my < y + h;
    }

    void draw(olc::PixelGameEngine *pge, int mx, int my, bool isPressed) const
    {
        olc::Pixel color = normalColor;
        if (isHovered(mx, my))
        {
            color = isPressed ? activeColor : hoverColor;
        }
        pge->FillRect(x, y, w, h, color);
        pge->DrawRect(x, y, w, h, olc::WHITE);

        olc::vi2d textSize = pge->GetTextSize(label);
        int tx = x + (w - textSize.x) / 2;
        int ty = y + (h - textSize.y) / 2;
        pge->DrawString(tx, ty, label, olc::WHITE);
    }
};

class SwerveVisualizer : public olc::PixelGameEngine
{
public:
    SwerveVisualizer()
    {
        sAppName = "Swerve Drive Simulator";
    }

private:
    std::unique_ptr<SwerveDrive> robot;
    std::unique_ptr<SwerveDriveUtil> driveUtil;

    float fScale = Constants::Simulation::SCALE_PX_PER_M;
    // Trail of robot positions for visualizing trajectory
    std::deque<Physics::Vector2> robotTrail;
    const size_t kMaxTrailSize = 200;
    // Start position for the first waypoint (captured once per routine)
    Physics::Vector2 startWaypointPos;
    bool startWaypointCaptured = false;
    olc::vf2d vOffset = {400, 400};
    float fSimStep = (float)Constants::Simulation::PHYSICS_DT;

    // UI Buttons
    std::vector<Button> buttons;

    // Click target for grid right-clicking (stored as state, re-submitted every frame)
    std::optional<Physics::Vector2> clickTarget;
    double clickTargetHeading{0.0};

    // Active autonomous command and display tracking
    std::shared_ptr<Command> m_activeAutonCommand;
    std::vector<AutonWaypoint> m_activeWaypoints;
    std::string m_activeAutonName{"None"};

public:
    bool OnUserCreate() override
    {
        robot = std::make_unique<SwerveDrive>(Pose{Vector2(0, 0), 0});
        driveUtil = std::make_unique<SwerveDriveUtil>(*robot);

        // Dashboard buttons (Row 1: S-Curve & Square, Row 2: P2P & Cancel)
        buttons = {
            {820, 580, 170, 32, "ZigZag Auton"},
            {1005, 580, 170, 32, "Square Auton"},
            {820, 625, 170, 32, "P2P Auton"},
            {1005, 625, 170, 32, "Cancel Auton"}};

        return true;
    }

    bool OnUserUpdate(float fElapsedTime) override
    {
        // INPUT HANDLING
        float targetVx = 0, targetVy = 0, targetOmega = 0;
        float speed = Constants::Simulation::DRIVE_SPEED_MPS;
        float rotSpeed = Constants::Simulation::ROTATE_SPEED_RADS;

        bool teleopInputActive = false;
        if (GetKey(olc::Key::UP).bHeld || GetKey(olc::Key::W).bHeld)
        {
            targetVx += speed;
            teleopInputActive = true;
        }
        if (GetKey(olc::Key::DOWN).bHeld || GetKey(olc::Key::S).bHeld)
        {
            targetVx -= speed;
            teleopInputActive = true;
        }
        if (GetKey(olc::Key::LEFT).bHeld || GetKey(olc::Key::A).bHeld)
        {
            targetVy += speed;
            teleopInputActive = true;
        }
        if (GetKey(olc::Key::RIGHT).bHeld || GetKey(olc::Key::D).bHeld)
        {
            targetVy -= speed;
            teleopInputActive = true;
        }
        if (GetKey(olc::Key::Q).bHeld)
        {
            targetOmega += rotSpeed;
            teleopInputActive = true;
        }
        if (GetKey(olc::Key::E).bHeld)
        {
            targetOmega -= rotSpeed;
            teleopInputActive = true;
        }

        // If manual controls are touched, cancel running commands
        if (teleopInputActive)
        {
            if (clickTarget.has_value())
            {
                clickTarget = std::nullopt;
                driveUtil->resetToIdle();
            }
            CommandScheduler::getInstance().cancelAll();
            m_activeAutonCommand.reset();
            m_activeWaypoints.clear();
            m_activeAutonName = "None";
        }

        // Emergency stop
        if (GetKey(olc::Key::X).bPressed)
        {
            CommandScheduler::getInstance().cancelAll();
            m_activeAutonCommand.reset();
            m_activeWaypoints.clear();
            m_activeAutonName = "None";
            clickTarget = std::nullopt;
            driveUtil->emergencyStop();
        }

        // Handle Dashboard Button Clicks
        if (GetMouse(0).bPressed)
        {
            int mx = GetMouseX();
            int my = GetMouseY();
            std::cout << "[DEBUG] Mouse Click in meters at (" << (mx / Constants::Simulation::SCALE_PX_PER_M) << ", " << (my / Constants::Simulation::SCALE_PX_PER_M) << ")\n";

            for (size_t i = 0; i < buttons.size(); i++)
            {
                if (buttons[i].isHovered(mx, my))
                {
                    clickTarget = std::nullopt;
                    CommandScheduler::getInstance().cancelAll();
                    m_activeAutonCommand.reset();
                    m_activeWaypoints.clear();
                    m_activeAutonName = "None";

                    if (i == 0)
                    {
                        auto r = std::make_unique<ZigZagRoutine>();
                        m_activeAutonName = r->getName();
                        m_activeWaypoints = r->getWaypoints();
                        m_activeAutonCommand = r->getCommand(robot.get(), driveUtil.get());
                        CommandScheduler::getInstance().schedule(m_activeAutonCommand);
                    }
                    else if (i == 1)
                    {
                        auto r = std::make_unique<SquareRoutine>();
                        m_activeAutonName = r->getName();
                        m_activeWaypoints = r->getWaypoints();
                        m_activeAutonCommand = r->getCommand(robot.get(), driveUtil.get());
                        CommandScheduler::getInstance().schedule(m_activeAutonCommand);
                    }
                    else if (i == 2)
                    {
                        auto r = std::make_unique<PointToPointRoutine>();
                        m_activeAutonName = r->getName();
                        m_activeWaypoints = r->getWaypoints();
                        m_activeAutonCommand = r->getCommand(robot.get(), driveUtil.get());
                        CommandScheduler::getInstance().schedule(m_activeAutonCommand);
                    }
                    else if (i == 3)
                    {
                        driveUtil->resetToIdle();
                    }
                }
            }
        }

        // Right Click on Grid to PID
        if (GetMouse(1).bPressed && GetMouseX() <= 800)
        {
            CommandScheduler::getInstance().cancelAll();
            m_activeAutonCommand.reset();
            m_activeWaypoints.clear();
            m_activeAutonName = "None";
            double worldX = (double)(GetMouseX() - vOffset.x) / fScale;
            double worldY = (double)(vOffset.y - GetMouseY()) / fScale;
            clickTarget = Physics::Vector2(worldX, worldY);
            clickTargetHeading = robot->getState().pose.rotation; // hold current heading
        }

        // Per-frame drive command priority:
        //   1. Auton (handled via command scheduler running in physics loop)
        //   2. Click-target PID
        //   3. Teleop
        if (!CommandScheduler::getInstance().hasActiveCommands())
        {
            if (clickTarget.has_value())
            {
                DriveRequest req;
                req.mode = DriveMode::TARGET_POSE;
                req.priority = RequestPriority::ALIGNMENT;
                req.translation = {0, 0};
                req.rotation = 0;
                req.targetPosition = clickTarget;
                req.targetHeading = clickTargetHeading;
                req.rampTimeSeconds = 0.0;
                req.source = "GRID_CLICK";
                req.isActive = true;
                driveUtil->submitRequest(req);
            }
            else
            {
                driveUtil->submitRequest(DriveRequest::teleop(targetVx, targetVy, targetOmega));
            }
        }

        // PHYSICS STEPPING
        static float fAccumulator = 0.0f;
        fAccumulator += fElapsedTime;

        // Caps to prevent da Spiral of Death
        if (fAccumulator > Constants::Simulation::MAX_ACCUMULATOR)
        {
            fAccumulator = Constants::Simulation::MAX_ACCUMULATOR;
        }

        while (fAccumulator >= fSimStep)
        {
            // Update scheduled commands
            CommandScheduler::getInstance().run(fSimStep);

            // Clean up visual references if command group finishes
            if (m_activeAutonCommand && !CommandScheduler::getInstance().hasActiveCommands())
            {
                m_activeAutonCommand.reset();
                m_activeWaypoints.clear();
                m_activeAutonName = "None";
            }

            driveUtil->update(fSimStep);
            robot->update(fSimStep);
            // Record robot position for trail
            robotTrail.push_back(robot->getState().pose.position);
            if (robotTrail.size() > kMaxTrailSize) robotTrail.pop_front();

            static int debugCounter = 0;
            debugCounter++;
            if (debugCounter % 50 == 0)
            {
                std::cout << "[DEBUG] Robot Pose: (" << robot->getState().pose.position.x
                          << ", " << robot->getState().pose.position.y << "), Head: "
                          << robot->getState().pose.rotation << "\n";
            }

            fAccumulator -= fSimStep;
        }

        // Drawing
        Clear(olc::Pixel(20, 20, 20));

        for (int x = 0; x <= 800; x += (int)fScale)
            DrawLine(x, 0, x, ScreenHeight(), olc::Pixel(45, 45, 45));
        for (int y = 0; y <= ScreenHeight(); y += (int)fScale)
            DrawLine(0, y, 800, y, olc::Pixel(45, 45, 45));

        DrawAutonPathAndTarget();
        DrawRobot();
        DrawDashboard();

        return true;
    }

private:
    void DrawRobot()
    {
        // Draw robot trajectory trail (behind robot)
        if (robotTrail.size() > 1) {
            size_t idx = 0;
            for (auto it = robotTrail.begin(); std::next(it) != robotTrail.end(); ++it, ++idx) {
                const auto& p1 = *it;
                const auto& p2 = *std::next(it);
                int x1 = p1.x * fScale + vOffset.x;
                int y1 = -p1.y * fScale + vOffset.y;
                int x2 = p2.x * fScale + vOffset.x;
                int y2 = -p2.y * fScale + vOffset.y;
                uint8_t alpha = static_cast<uint8_t>(255 * (static_cast<float>(idx) / robotTrail.size()));
                olc::Pixel trailColor = olc::Pixel(0, 255, 0, alpha);
                DrawLine(x1, y1, x2, y2, trailColor);
            }
        }
        // Draw robot on top of trail
        robot->draw(this, fScale, vOffset);
    }


    void DrawAutonPathAndTarget()
    {
        if (!m_activeWaypoints.empty())
        {
            if (!startWaypointCaptured)
            {
                startWaypointPos = robot->getState().pose.position;
                startWaypointCaptured = true;
            }
            // Convert stored start position to screen coordinates
            int robotScreenX = startWaypointPos.x * fScale + vOffset.x;
            int robotScreenY = -startWaypointPos.y * fScale + vOffset.y;

            for (size_t i = 0; i < m_activeWaypoints.size(); i++)
            {
                int wx, wy;
                if (i == 0) {
                    wx = robotScreenX;
                    wy = robotScreenY;
                } else {
                    wx = m_activeWaypoints[i].position.x * fScale + vOffset.x;
                    wy = -m_activeWaypoints[i].position.y * fScale + vOffset.y;
                }

                // Waypoint nodes (different color for start)
                if (i == 0) {
                    FillCircle(wx, wy, 5, olc::Pixel(0, 255, 0)); // bright green start marker
                    DrawCircle(wx, wy, 10, olc::Pixel(0, 255, 0));
                    DrawString(wx + 12, wy - 12, "W0", olc::Pixel(0, 255, 0));
                } else {
                    FillCircle(wx, wy, 4, olc::Pixel(255, 165, 0)); // orange/tangerine
                    DrawCircle(wx, wy, 8, olc::Pixel(255, 165, 0));
                    DrawString(wx + 10, wy - 10, "W" + std::to_string(i), olc::Pixel(255, 165, 0));
                }

                // Line connecting to next waypoint
                if (i < m_activeWaypoints.size() - 1)
                {
                    int nwx = m_activeWaypoints[i + 1].position.x * fScale + vOffset.x;
                    int nwy = -m_activeWaypoints[i + 1].position.y * fScale + vOffset.y;
                    DrawLine(wx, wy, nwx, nwy, olc::Pixel(255, 165, 0, 100));
                }
            }
        }
        else
        {
            startWaypointCaptured = false;
        }

        // right-click target position
        if (clickTarget.has_value())
        {
            int tx = clickTarget->x * fScale + vOffset.x;
            int ty = -clickTarget->y * fScale + vOffset.y;

            // Glowing cyan crosshair and target circles
            DrawCircle(tx, ty, 6, olc::CYAN);
            DrawCircle(tx, ty, 2, olc::CYAN);
            DrawLine(tx - 10, ty, tx + 10, ty, olc::CYAN);
            DrawLine(tx, ty - 10, tx, ty + 10, olc::CYAN);
            DrawString(tx + 12, ty - 12, "TARGET", olc::CYAN);

            const auto& robotPos = robot->getState().pose.position;
            int rx = robotPos.x * fScale + vOffset.x;
            int ry = -robotPos.y * fScale + vOffset.y;
            DrawLine(rx, ry, tx, ty, olc::Pixel(0, 255, 255, 120));
        }
    }

    void DrawDashboard()
    {
        int x = 820;
        DrawString(x, 20, "Swerve Standalone", olc::WHITE, 2);

        DrawString(x, 60, "Controls:", olc::GREY);
        DrawString(x, 80, "Arrows: Drive  Q/E: Rotate  X: Stop", olc::WHITE);
        DrawString(x, 95, "Right Click Grid: Go to Target", olc::WHITE);

        auto smoothedTrans = driveUtil->getSmoothedTranslation();
        double smoothedRot = driveUtil->getSmoothedRotation();
        DrawString(x, 130, "Drive Input (Smoothed):", olc::CYAN);
        DrawString(x, 150, "Vx: " + std::to_string(smoothedTrans.x), olc::WHITE);
        DrawString(x, 165, "Vy: " + std::to_string(smoothedTrans.y), olc::WHITE);
        DrawString(x, 180, "Rot: " + std::to_string(smoothedRot), olc::WHITE);

        auto state = robot->getState();
        DrawString(x, 210, "Robot State:", olc::GREEN);
        DrawString(x, 230, "X: " + std::to_string(state.pose.position.x), olc::WHITE);
        DrawString(x, 245, "Y: " + std::to_string(state.pose.position.y), olc::WHITE);
        DrawString(x, 260, "H: " + std::to_string(state.pose.rotation * 180.0 / M_PI), olc::WHITE);

        DrawString(x, 290, "Modules (Drive V / Steer V):", olc::YELLOW);
        const char *names[] = {"FL", "FR", "BL", "BR"};
        auto &voltages = robot->getModuleVoltages();
        for (int i = 0; i < 4; i++)
        {
            int y = 310 + i * 15;
            DrawString(x, y, std::string(names[i]) + ": " + std::to_string(voltages[i].first).substr(0, 5) + "V / " + std::to_string(voltages[i].second).substr(0, 5) + "V");
        }

        DrawString(x, 380, "Request System:", olc::Pixel(180, 130, 255));
        auto curReq = driveUtil->getCurrentRequest();
        DrawString(x, 400, "Source: " + curReq.source, olc::WHITE);
        DrawString(x, 415, "Priority: " + std::to_string(static_cast<int>(curReq.priority)), olc::WHITE);
        DrawString(x, 430, "Queue: " + std::to_string(driveUtil->queueSize()), olc::WHITE);
        int blendPct = static_cast<int>(driveUtil->getTransitionProgress() * 100);
        DrawString(x, 445, "Blend: " + std::to_string(blendPct) + "%", olc::WHITE);

        bool running = (m_activeAutonCommand != nullptr);
        olc::Pixel autoColor = running ? olc::Pixel(120, 255, 120) : olc::Pixel(120, 120, 120);
        DrawString(x, 480, "Autonomous & Control UI:", olc::Pixel(255, 200, 80));

        std::string statusStr = "None";
        if (running)
        {
            statusStr = m_activeAutonName + " [" + m_activeAutonCommand->getStatus() + "]";
        }
        DrawString(x, 500, "Routine: " + statusStr, autoColor);
        if (running)
        {
            double progress = m_activeAutonCommand->getProgress();
            int pct = static_cast<int>(progress * 100);
            DrawString(x, 515, "Progress: " + std::to_string(pct) + "%", autoColor);
            int barW = 350;
            int barH = 8;
            int barX = x;
            int barY = 532;
            DrawRect(barX, barY, barW, barH, olc::Pixel(80, 80, 80));
            FillRect(barX + 1, barY + 1, static_cast<int>(barW * progress), barH - 1, autoColor);
        }

        // buttons
        int mx = GetMouseX();
        int my = GetMouseY();
        bool isPressed = GetMouse(0).bHeld;
        for (const auto &btn : buttons)
        {
            btn.draw(this, mx, my, isPressed);
        }
    }
};

int main()
{
    SwerveVisualizer demo;
    if (demo.Construct(1200, 800, 1, 1))
        demo.Start();
    return 0;
}

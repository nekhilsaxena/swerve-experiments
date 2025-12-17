#define OLC_PGE_APPLICATION
#include "olcPixelGameEngine.h"
#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <string>
#include <iomanip>

#include "Physics.h"
#include "SwerveModule.h"
#include "SwerveDrive.h"

using namespace Physics;

class SwerveVisualizer : public olc::PixelGameEngine {
public:
    SwerveVisualizer() {
        sAppName = "Swerve Drive Simulator";
    }

private:
    std::unique_ptr<SwerveDrive> robot;
    SwerveDrive::Config driveConfig;
    std::vector<SwerveModule::Config> moduleConfigs;
    
    float fScale = 100.0f;
    olc::vf2d vOffset = {400, 400};

    struct InputState {
        float vx = 0;
        float vy = 0;
        float omega = 0;
    } currentInput;

    std::vector<std::pair<double, double>> currentControls;

public:
    bool OnUserCreate() override {
        // 1. Robot Setup (Identical physics setup)
        DCMotor driveMotor(6000, 4.69, 255, 1.5, 12.0); 
        DCMotor steerMotor(6000, 2.0, 100, 1.5, 12.0); 

        SwerveModule::Config modConfig;
        modConfig.driveMotor = driveMotor;
        modConfig.steerMotor = steerMotor;
        modConfig.driveGearRatio = 6.75;
        modConfig.steerGearRatio = 12.8; 
        modConfig.wheelRadius = 0.0508; 
        modConfig.massOnWheel = 50.0 / 4.0; 
        modConfig.wheelMOI = 0.005; 
        modConfig.moduleMOI = 0.05; 
        modConfig.coffFriction = 1.1; 
        modConfig.steerFriction = 0.1; 
        modConfig.position = Vector2(0,0);

        double hw = 0.6 / 2.0; 
        moduleConfigs.assign(4, modConfig);
        moduleConfigs[0].position = Vector2(hw, hw);   // FL
        moduleConfigs[1].position = Vector2(hw, -hw);  // FR
        moduleConfigs[2].position = Vector2(-hw, hw);  // BL
        moduleConfigs[3].position = Vector2(-hw, -hw); // BR

        driveConfig.mass = 50.0;
        driveConfig.moi = 5.0; 
        driveConfig.modules = moduleConfigs;

        robot = std::make_unique<SwerveDrive>(driveConfig, Pose{Vector2(0,0), 0});
        currentControls.assign(4, {0.0, 0.0});

        return true;
    }

    bool OnUserUpdate(float fElapsedTime) override {
        // --- INPUT HANDLING ---
        float targetVx = 0, targetVy = 0, targetOmega = 0;
        float speed = 2.5f;
        float rotSpeed = 3.0f;

        if (GetKey(olc::Key::UP).bHeld) targetVx += speed;
        if (GetKey(olc::Key::DOWN).bHeld) targetVx -= speed;
        if (GetKey(olc::Key::LEFT).bHeld) targetVy += speed;
        if (GetKey(olc::Key::RIGHT).bHeld) targetVy -= speed;
        if (GetKey(olc::Key::Q).bHeld) targetOmega += rotSpeed;
        if (GetKey(olc::Key::E).bHeld) targetOmega -= rotSpeed;
        if (GetKey(olc::Key::X).bPressed) { targetVx = 0; targetVy = 0; targetOmega = 0; }

        // Simple Ramping / Input Smoothing for a "Complete Project" feel
        float lerpRate = 15.0f * fElapsedTime; 
        currentInput.vx += (targetVx - currentInput.vx) * std::min(1.0f, lerpRate);
        currentInput.vy += (targetVy - currentInput.vy) * std::min(1.0f, lerpRate);
        currentInput.omega += (targetOmega - currentInput.omega) * std::min(1.0f, lerpRate);

        // --- PHYSICS STEPPING ---
        float fSimStep = 0.01f;
        static float fAccumulator = 0.0f;
        fAccumulator += fElapsedTime;

        // Caps to prevent "Spiral of Death" if debugger is paused or lag occurs
        if (fAccumulator > 0.1f) fAccumulator = 0.1f;

        while (fAccumulator >= fSimStep) {
            UpdatePhysics(fSimStep);
            fAccumulator -= fSimStep;
        }

        // --- DRAWING ---
        Clear(olc::Pixel(20, 20, 20));

        // 1. Grid
        for (int x = 0; x <= 800; x += (int)fScale) DrawLine(x, 0, x, ScreenHeight(), olc::Pixel(45, 45, 45));
        for (int y = 0; y <= ScreenHeight(); y += (int)fScale) DrawLine(0, y, 800, y, olc::Pixel(45, 45, 45));

        // 2. Robot
        DrawRobot();

        // 3. UI Dashboard
        DrawDashboard();

        return true;
    }

private:
    void UpdatePhysics(float dt) {
        double angle = robot->getState().pose.rotation;
        double cosA = std::cos(-angle);
        double sinA = std::sin(-angle);
        
        // Field oriented transformation
        double sigVx = currentInput.vx * cosA - currentInput.vy * sinA;
        double sigVy = currentInput.vx * sinA + currentInput.vy * cosA;
        
        auto& modules = robot->getModules();
        static double lastDesAngle[4] = {0, 0, 0, 0};

        for (size_t i = 0; i < 4; ++i) {
            Vector2 r = moduleConfigs[i].position;
            Vector2 v_rot(-r.y, r.x);
            Vector2 v_mod(sigVx, sigVy);
            v_mod = v_mod + v_rot * currentInput.omega;

            double desSpeed = v_mod.magnitude();
            double desAngle = lastDesAngle[i];

            // DEAD BAND & ANGLE LATCHING:
            // Only update steering angle if we are actually trying to move or if the module is spinning fast.
            // This prevents the "jitter" where modules snap to 0 deg when robot stops.
            if (desSpeed > 0.05) {
                desAngle = std::atan2(v_mod.y, v_mod.x);
                lastDesAngle[i] = desAngle;
            }

            double curAngle = modules[i].getState().steerAngle;
            
            // Standard Swerve Optimization (find shortest path)
            double delta = desAngle - curAngle;
            while (delta > M_PI) delta -= 2 * M_PI;
            while (delta < -M_PI) delta += 2 * M_PI;
            
            if (std::abs(delta) > M_PI / 2.0) {
                desAngle += M_PI;
                desSpeed *= -1.0;
            }
            
            double error = desAngle - curAngle;
            while (error > M_PI) error -= 2 * M_PI;
            while (error < -M_PI) error += 2 * M_PI;

            // PID Tuning (Simple P for now)
            double steerV = error * 15.0; // Slightly stiffer steering
            double driveV = desSpeed * 2.8; 
            
            // DEAD BAND for voltages to stop resting jitter
            if (std::abs(driveV) < 0.1) driveV = 0.0;
            if (std::abs(steerV) < 0.05) steerV = 0.0;

            // Clamp Physical Limits
            if (steerV > 12) steerV = 12; if (steerV < -12) steerV = -12;
            if (driveV > 12) driveV = 12; if (driveV < -12) driveV = -12;

            currentControls[i] = {driveV, steerV};
        }

        robot->update(dt, currentControls);
    }

    void DrawRobot() {
        auto state = robot->getState();
        olc::vf2d pos = { (float)state.pose.position.x * fScale, (float)-state.pose.position.y * fScale };
        pos += vOffset;

        float angle = (float)-state.pose.rotation;
        float size = 0.6f * fScale;

        auto Rotate = [&](float x, float y, float a) {
            return olc::vf2d{ x * cosf(a) - y * sinf(a), x * sinf(a) + y * cosf(a) };
        };

        // Draw Body Square
        olc::vf2d p1 = pos + Rotate(-size / 2, -size / 2, angle);
        olc::vf2d p2 = pos + Rotate(size / 2, -size / 2, angle);
        olc::vf2d p3 = pos + Rotate(size / 2, size / 2, angle);
        olc::vf2d p4 = pos + Rotate(-size / 2, size / 2, angle);

        DrawLine(p1, p2, olc::Pixel(78, 201, 176));
        DrawLine(p2, p3, olc::Pixel(78, 201, 176));
        DrawLine(p3, p4, olc::Pixel(78, 201, 176));
        DrawLine(p4, p1, olc::Pixel(78, 201, 176));
        
        // Heading arrow
        olc::vf2d head = pos + Rotate(size / 2, 0, angle);
        DrawLine(pos, head, olc::WHITE);

        // Modules
        auto& modules = robot->getModules();
        for (int i = 0; i < 4; i++) {
            olc::vf2d modPos = pos + Rotate((float)moduleConfigs[i].position.x * fScale, (float)-moduleConfigs[i].position.y * fScale, angle);
            float steer = (float)-modules[i].getState().steerAngle + angle;
            
            olc::vf2d w1 = modPos + Rotate(-10, -5, steer);
            olc::vf2d w2 = modPos + Rotate(10, -5, steer);
            olc::vf2d w3 = modPos + Rotate(10, 5, steer);
            olc::vf2d w4 = modPos + Rotate(-10, 5, steer);
            
            DrawLine(w1, w2, olc::Pixel(206, 145, 120));
            DrawLine(w2, w3, olc::Pixel(206, 145, 120));
            DrawLine(w3, w4, olc::Pixel(206, 145, 120));
            DrawLine(w4, w1, olc::Pixel(206, 145, 120));

            // Speed vector
            olc::vf2d vec = Rotate((float)modules[i].getState().wheelSpeed * 10.0f, 0, steer);
            DrawLine(modPos, modPos + vec, olc::Pixel(220, 220, 170));
        }
    }

    void DrawDashboard() {
        int x = 820;
        DrawString(x, 20, "Swerve Standalone", olc::WHITE, 2);
        
        DrawString(x, 60, "Controls:", olc::GREY);
        DrawString(x, 80, "Arrows: Move, Q/E: Rotate, X: Stop", olc::WHITE);

        DrawString(x, 120, "User Input:", olc::CYAN);
        DrawString(x, 140, "Vx: " + std::to_string(currentInput.vx), olc::WHITE);
        DrawString(x, 160, "Vy: " + std::to_string(currentInput.vy), olc::WHITE);
        DrawString(x, 180, "Rot: " + std::to_string(currentInput.omega), olc::WHITE);

        auto state = robot->getState();
        DrawString(x, 220, "Robot State:", olc::GREEN);
        DrawString(x, 240, "X: " + std::to_string(state.pose.position.x), olc::WHITE);
        DrawString(x, 260, "Y: " + std::to_string(state.pose.position.y), olc::WHITE);
        DrawString(x, 280, "H: " + std::to_string(state.pose.rotation * 180.0 / M_PI), olc::WHITE);

        DrawString(x, 320, "Modules (Drive V / Steer V):", olc::YELLOW);
        const char* names[] = {"FL", "FR", "BL", "BR"};
        for (int i = 0; i < 4; i++) {
            int y = 340 + i * 20;
            DrawString(x, y, std::string(names[i]) + ": " + std::to_string(currentControls[i].first).substr(0,4) + "V / " + std::to_string(currentControls[i].second).substr(0,4) + "V");
        }
    }
};

int main() {
    SwerveVisualizer demo;
    if (demo.Construct(1200, 800, 1, 1))
        demo.Start();
    return 0;
}

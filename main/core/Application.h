#pragma once

#ifndef GNSSLAB_APPLICATION_H
#define GNSSLAB_APPLICATION_H

#include "ui/Gui.h"

class Application {
public:
    Application();
    void Run();

private:
    void Initialize();
    void Shutdown();
    void Update();
    void Render();

    Gui   m_ui;
    bool  m_showTimeConverter  = false;
    bool  m_showCoordConverter = false;
};

#endif

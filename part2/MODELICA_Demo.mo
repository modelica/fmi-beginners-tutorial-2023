package MODELICA_Demo
    extends Modelica.Icons.Package;
  model Stimuli
    Modelica.Blocks.Sources.Step stepTau(startTime = 0.5, height = 3) annotation (
      Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 0, origin = {-30, -50})));
    Modelica.Blocks.Sources.Step stepW(startTime = 0.1, height = 10) annotation (
      Placement(transformation(extent = {{-40, 40}, {-20, 60}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput w_desired annotation (
      Placement(transformation(extent = {{100, 40}, {120, 60}}), iconTransformation(extent = {{100, 40}, {120, 60}})));
    Modelica.Blocks.Interfaces.RealOutput LoadTorque_Nm annotation (
      Placement(transformation(extent = {{100, -60}, {120, -40}}), iconTransformation(extent = {{100, -60}, {120, -40}})));
  equation
    connect(stepW.y, w_desired) annotation (
      Line(points = {{-19, 50}, {110, 50}}, color = {0, 0, 127}, smooth = Smooth.None));
    connect(stepTau.y, LoadTorque_Nm) annotation (
      Line(points = {{-19, -50}, {110, -50}}, color = {0, 0, 127}, smooth = Smooth.None));
    annotation (
      Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})),
      Icon(graphics={  Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {28, 108, 200}, fillColor = {170, 255, 255},
              fillPattern =                                                                                                                  FillPattern.Solid), Text(extent = {{-60, 60}, {100, 40}}, lineColor = {28, 108, 200}, textString = "w_desired"), Text(extent = {{-36, -40}, {100, -60}}, lineColor = {28, 108, 200}, textString = "tau_load")}));
  end Stimuli;

  model Control
    Modelica.Blocks.Interfaces.RealInput w_desired annotation (
      Placement(visible = true,transformation(extent = {{-140, -20}, {-100, 20}}, rotation = 0), iconTransformation(extent = {{-140, 20}, {-100, 60}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput V annotation (
      Placement(transformation(extent = {{90, -10}, {110, 10}}), iconTransformation(extent = {{100, -10}, {120, 10}})));
    Modelica.Blocks.Interfaces.RealInput w annotation (
      Placement(visible = true,transformation(extent = {{-140, -100}, {-100, -60}}, rotation = 0), iconTransformation( origin = {-120, -40},extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Math.Feedback speederror annotation (
      Placement(transformation(extent = {{-30, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Blocks.Continuous.PI PI(k = 0.1, T = 0.005, initType = Modelica.Blocks.Types.Init.InitialOutput) annotation (
      Placement(transformation(extent = {{20, -10}, {40, 10}})));
  equation
    connect(w, speederror.u2) annotation (
      Line(points = {{-120, -80}, {-20, -80}, {-20, -8}}, color = {0, 0, 127}));
    connect(w_desired, speederror.u1) annotation (
      Line(points = {{-120, 0}, {-28, 0}}, color = {0, 0, 127}, smooth = Smooth.None));
    connect(speederror.y, PI.u) annotation (
      Line(points = {{-11, 0}, {18, 0}}, color = {0, 0, 127}, smooth = Smooth.None));
    connect(PI.y, V) annotation (
      Line(points = {{41, 0}, {100, 0}}, color = {0, 0, 127}, smooth = Smooth.None));
    annotation (
      Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})),
      Icon(coordinateSystem(preserveAspectRatio = false, initialScale = 0.1), graphics={  Text(origin = {0, 40}, lineColor = {0, 0, 255}, extent = {{-180, 40}, {-100, 20}}, textString = "w_desired"), Text(origin = {-160, 60},lineColor = {0, 0, 255}, extent = {{20, -60}, {40, -80}}, textString = "w"), Text(lineColor = {0, 0, 255}, extent = {{100, 40}, {120, 20}}, textString = "V"), Rectangle(lineColor = {0, 0, 255}, fillColor = {213, 255, 170},
              fillPattern =                                                                                                                                                                                                        FillPattern.Solid, extent = {{-100, 60}, {100, -60}}), Text(origin = {0, -56},lineColor = {0, 0, 255}, extent = {{60, 50}, {-60, 70}}, textString = "%name")}));
  end Control;

  model Drive
    Modelica.Blocks.Interfaces.RealInput V annotation (
      Placement(transformation(extent = {{-120, -10}, {-100, 10}}), iconTransformation(extent={{-140,20},
              {-100,60}})));
    Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage annotation (
      Placement(transformation(extent = {{-20, 20}, {-40, 40}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Ground ground annotation (
      Placement(transformation(origin = {-50, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));
    Modelica.Mechanics.Rotational.Components.Inertia loadInertia1(J = 1, phi(fixed = true, start = 0), w(fixed = true, start = 0)) annotation (
      Placement(transformation(extent = {{30, -10}, {50, 10}}, rotation = 0)));
    Modelica.Mechanics.Rotational.Components.IdealGear idealGear(ratio = 10) annotation (
      Placement(transformation(extent = {{0, -10}, {20, 10}})));
    Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor annotation (
      Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {0, -30})));
    Modelica.Blocks.Interfaces.RealOutput w annotation (
      Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {0, -100}), iconTransformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin={0,-70})));
    Modelica.Mechanics.Rotational.Sources.Torque torque annotation (
      Placement(transformation(extent = {{10, -10}, {-10, 10}}, rotation = 0, origin = {70, 0})));
    Modelica.Blocks.Interfaces.RealInput LoadTorque_Nm annotation (
      Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 180, origin = {110, 0}), iconTransformation(extent={{20,-20},
              {-20,20}},                                                                                                                                rotation = 180, origin={-120,-40})));
    Modelica.Electrical.Machines.BasicMachines.DCMachines.DC_PermanentMagnet dcpm(VaNominal = dcpmData.VaNominal, IaNominal = dcpmData.IaNominal, wNominal = dcpmData.wNominal, TaNominal = dcpmData.TaNominal, Ra = dcpmData.Ra, TaRef = dcpmData.TaRef, La = dcpmData.La, Jr = dcpmData.Jr, useSupport = false, Js = dcpmData.Js, frictionParameters = dcpmData.frictionParameters, coreParameters = dcpmData.coreParameters, strayLoadParameters = dcpmData.strayLoadParameters, brushParameters = dcpmData.brushParameters, TaOperational = 293.15, alpha20a = dcpmData.alpha20a, phiMechanical(fixed = false), wMechanical(fixed = false), ia(fixed = true)) annotation (
      Placement(transformation(extent = {{-40, -10}, {-20, 10}})));
    parameter Modelica.Electrical.Machines.Utilities.ParameterRecords.DcPermanentMagnetData dcpmData(Jr = 0.001, wNominal = 149.22565104552, brushParameters(V = 0.7)) annotation (
      Placement(transformation(extent = {{-40, -40}, {-20, -20}})));
  equation
    connect(ground.p, signalVoltage.n) annotation (
      Line(points = {{-40, 30}, {-40, 30}}, color = {0, 0, 255}, smooth = Smooth.None));
    connect(V, signalVoltage.v) annotation (
      Line(points={{-110,0},{-70,0},{-70,50},{-30,50},{-30,42}},            color = {0, 0, 127}, smooth = Smooth.None));
    connect(idealGear.flange_b, loadInertia1.flange_a) annotation (
      Line(points = {{20, 0}, {30, 0}}, color = {0, 0, 0}, smooth = Smooth.None));
    connect(idealGear.flange_a, speedSensor.flange) annotation (
      Line(points = {{0, 0}, {1.77636e-15, 0}, {1.77636e-15, -20}}, color = {0, 0, 0}, smooth = Smooth.None));
    connect(speedSensor.w, w) annotation (
      Line(points = {{-1.9984e-15, -41}, {0, -41}, {0, -100}}, color = {0, 0, 127}, smooth = Smooth.None));
    connect(torque.flange, loadInertia1.flange_b) annotation (
      Line(points = {{60, 0}, {50, 0}}, color = {0, 0, 0}, smooth = Smooth.None));
    connect(LoadTorque_Nm, torque.tau) annotation (
      Line(points = {{110, -4.44089e-16}, {94, -4.44089e-16}, {94, 0}, {82, 0}}, color = {0, 0, 127}, smooth = Smooth.None));
    connect(dcpm.flange, idealGear.flange_a) annotation (
      Line(points = {{-20, 0}, {0, 0}}, color = {0, 0, 0}));
    connect(dcpm.pin_an, ground.p) annotation (
      Line(points = {{-36, 10}, {-40, 10}, {-40, 30}}, color = {0, 0, 255}));
    connect(dcpm.pin_ap, signalVoltage.p) annotation (
      Line(points = {{-24, 10}, {-20, 10}, {-20, 30}}, color = {0, 0, 255}));
    annotation (
      Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})),
      Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics={  Text(extent={{
                -140,80},{-120,60}},                                                                                                               lineColor = {0, 0, 255}, textString = "V"), Rectangle(extent={{
                -100,60},{100,-60}},                                                                                                                                                                                                        lineColor = {0, 0, 255}, fillColor = {170, 213, 255},
              fillPattern =                                                                                                                                                                                                        FillPattern.Solid), Text(extent={{
                10,-60},{30,-80}},                                                                                                                                                                                                        lineColor = {0, 0, 255}, textString = "w"), Text(extent={{
                58,-10},{-62,10}},                                                                                                                                                                                                        lineColor = {0, 0, 255}, textString = "%name"), Text(extent={{
                -140,0},{-100,-20}},                                                                                                                                                                                                        lineColor = {0, 0, 255}, textString = "tau")}));
  end Drive;

  model ControlledElectricDrive
    extends Modelica.Icons.Example;
    Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage annotation (
      Placement(visible = true, transformation(extent = {{0, 20}, {-20, 40}}, rotation = 0)));
    Modelica.Electrical.Analog.Basic.Ground ground annotation (
      Placement(visible = true, transformation(origin = {-30, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));
    Modelica.Blocks.Math.Feedback speederror annotation (
      Placement(visible = true, transformation(extent = {{-80, -10}, {-60, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Step stepV(startTime = 0.1, height = 10) annotation (
      Placement(transformation(extent = {{-100, -10}, {-80, 10}}, rotation = 0)));
    Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor annotation (
      Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {10, -30})));
    Modelica.Mechanics.Rotational.Components.IdealGear idealGear(ratio = 10) annotation (
      Placement(transformation(extent = {{10, -10}, {30, 10}})));
    Modelica.Mechanics.Rotational.Sources.Torque torque annotation (
      Placement(transformation(extent = {{10, -10}, {-10, 10}}, rotation = 0, origin = {60, 0})));
    Modelica.Blocks.Sources.Step stepTau(startTime = 0.5, height = 3) annotation (
      Placement(transformation(extent = {{10, -10}, {-10, 10}}, rotation = 0, origin = {90, 0})));
    Modelica.Blocks.Continuous.PI PI(k = 0.1, T = 0.005, initType = Modelica.Blocks.Types.Init.InitialOutput) annotation (
      Placement(transformation(extent = {{-60, -10}, {-40, 10}})));
    Modelica.Mechanics.Rotational.Components.Inertia loadInertia1(J = 1, phi(start = 0, fixed = true), w(start = 0, fixed = true)) annotation (
      Placement(transformation(extent = {{30, -10}, {50, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput w annotation (
      Placement(transformation(extent = {{90, -70}, {110, -50}})));
    Modelica.Electrical.Machines.BasicMachines.DCMachines.DC_PermanentMagnet dcpm(VaNominal = dcpmData.VaNominal, IaNominal = dcpmData.IaNominal, wNominal = dcpmData.wNominal, TaNominal = dcpmData.TaNominal, Ra = dcpmData.Ra, TaRef = dcpmData.TaRef, La = dcpmData.La, Jr = dcpmData.Jr, useSupport = false, Js = dcpmData.Js, frictionParameters = dcpmData.frictionParameters, coreParameters = dcpmData.coreParameters, strayLoadParameters = dcpmData.strayLoadParameters, brushParameters = dcpmData.brushParameters, TaOperational = 293.15, alpha20a = dcpmData.alpha20a, phiMechanical(fixed = false), wMechanical(fixed = false), ia(fixed = true)) annotation (
      Placement(transformation(extent = {{-20, -10}, {0, 10}})));
    parameter Modelica.Electrical.Machines.Utilities.ParameterRecords.DcPermanentMagnetData dcpmData(Jr = 0.001, wNominal = 149.22565104552, brushParameters(V = 0.7)) annotation (
      Placement(transformation(extent = {{-20, -40}, {0, -20}})));
  equation
    connect(speederror.y, PI.u) annotation (
      Line(points = {{-61, 0}, {-62, 0}}, color = {0, 0, 127}));
    connect(speedSensor.w, speederror.u2) annotation (
      Line(points = {{10, -41}, {10, -60}, {-70, -60}, {-70, -8}}, color = {0, 0, 127}));
    connect(stepV.y, speederror.u1) annotation (
      Line(points = {{-79, 0}, {-78, 0}}, color = {0, 0, 127}));
    connect(PI.y, signalVoltage.v) annotation (
      Line(points={{-39,0},{-36.5,0},{-36.5,50},{-10,50},{-10,42}},            color = {0, 0, 127}));
    connect(ground.p, signalVoltage.n) annotation (
      Line(points = {{-20, 30}, {-20, 30}}, color = {0, 0, 255}));
    connect(stepTau.y, torque.tau) annotation (
      Line(points = {{79, 0}, {72, 0}}, color = {0, 0, 127}, smooth = Smooth.None));
    connect(idealGear.flange_b, loadInertia1.flange_a) annotation (
      Line(points = {{30, 0}, {30, 0}}, color = {0, 0, 0}, smooth = Smooth.None));
    connect(loadInertia1.flange_b, torque.flange) annotation (
      Line(points = {{50, 0}, {50, 0}}, color = {0, 0, 0}, smooth = Smooth.None));
    connect(speedSensor.w, w) annotation (
      Line(points = {{10, -41}, {10, -60}, {100, -60}}, color = {0, 0, 127}));
    connect(signalVoltage.p, dcpm.pin_ap) annotation (
      Line(points = {{0, 30}, {0, 10}, {-4, 10}}, color = {0, 0, 255}));
    connect(dcpm.pin_an, signalVoltage.n) annotation (
      Line(points = {{-16, 10}, {-20, 10}, {-20, 30}}, color = {0, 0, 255}));
    connect(dcpm.flange, idealGear.flange_a) annotation (
      Line(points = {{0, 0}, {10, 0}}, color = {0, 0, 0}));
    connect(speedSensor.flange, idealGear.flange_a) annotation (
      Line(points = {{10, -20}, {10, 0}}, color = {0, 0, 0}));
    annotation (
      Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})));
  end ControlledElectricDrive;

  model ControlledElectricDrive_subsystems
    extends Modelica.Icons.Example;
    Drive drive annotation (
      Placement(transformation(extent={{40,0},{80,40}})));
    Control control "control" annotation (
      Placement(transformation(extent = {{-30, 0}, {10, 40}})));
    Stimuli stimuli1 annotation (
      Placement(transformation(extent={{-92,-20},{-60,12}})));
    Modelica.Blocks.Interfaces.RealOutput w annotation (
      Placement(transformation(extent = {{90, -70}, {110, -50}})));
  equation
    connect(drive.w, w) annotation (
      Line(points={{60,6},{60,-60},{100,-60}},                             color = {0, 0, 127}));
    connect(control.V, drive.V) annotation (
      Line(points={{12,20},{24,20},{24,28},{36,28}},
                                          color = {0, 0, 127}, smooth = Smooth.None));
    connect(stimuli1.w_desired, control.w_desired) annotation (
      Line(points={{-58.4,4},{-49.17,4},{-49.17,28},{-34,28}},              color = {0, 0, 127}));
    connect(stimuli1.LoadTorque_Nm, drive.LoadTorque_Nm) annotation (
      Line(points={{-58.4,-12},{26,-12},{26,12},{36,12}},                                    color = {0, 0, 127}));
    connect(drive.w, control.w) annotation (
      Line(points={{60,6},{60,0},{-44,0},{-44,12},{-34,12}},      color = {0, 0, 127}));
    annotation (
      Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})));
  end ControlledElectricDrive_subsystems;

  model ControlledElectricDrive_subsystems_replaceable
    extends Modelica.Icons.Example;
    replaceable
    Drive drive annotation (
      Placement(transformation(extent={{40,0},{80,40}})));
    replaceable
    Control control "control" annotation (
      Placement(transformation(extent = {{-30, 0}, {10, 40}})));
    replaceable
    Stimuli stimuli1 annotation (
      Placement(transformation(extent={{-92,-20},{-60,12}})));
    Modelica.Blocks.Interfaces.RealOutput w annotation (
      Placement(transformation(extent = {{90, -70}, {110, -50}})));
  equation
    connect(drive.w, w) annotation (
      Line(points={{60,6},{60,-60},{100,-60}},                             color = {0, 0, 127}));
    connect(control.V, drive.V) annotation (
      Line(points={{12,20},{24,20},{24,28},{36,28}},
                                          color = {0, 0, 127}, smooth = Smooth.None));
    connect(drive.w, control.w) annotation (
      Line(points={{60,6},{60,-20},{-44,-20},{-44,12},{-34,12}},  color = {0, 0, 127}));
    connect(stimuli1.w_desired, control.w_desired) annotation (
      Line(points={{-58.4,4},{-49.17,4},{-49.17,28},{-34,28}},              color = {0, 0, 127}));
    connect(stimuli1.LoadTorque_Nm, drive.LoadTorque_Nm) annotation (
      Line(points={{-58.4,-12},{26,-12},{26,12},{36,12}},                                    color = {0, 0, 127}));
    annotation (
      Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})));
  end ControlledElectricDrive_subsystems_replaceable;

  model ControlledElectricDrive_fmu
    extends Modelica.Icons.Example;
    Modelica.Blocks.Interfaces.RealOutput w annotation (
      Placement(transformation(extent = {{90, -70}, {110, -50}})));
  MODELICA_Demo_ControlledElectricDrive_me_FMU mODELICA_Demo_ControlledElectricDrive_me_FMU1 annotation (
      Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(mODELICA_Demo_ControlledElectricDrive_me_FMU1.w, w) annotation (
      Line(points = {{12, 8}, {62, 8}, {62, -60}, {100, -60}, {100, -60}}, color = {0, 0, 127}));
    annotation (
      Icon(coordinateSystem(preserveAspectRatio = false)),
      Diagram(coordinateSystem(preserveAspectRatio = false)));
  end ControlledElectricDrive_fmu;

  model ControlledElectricDrive_ctrl_fmu_sl
    extends Modelica.Icons.Example;
    Drive drive annotation (
      Placement(transformation(extent={{40,0},{80,40}})));
    Stimuli stimuli1 annotation (
      Placement(transformation(extent={{-92,-20},{-60,12}})));
    Modelica.Blocks.Interfaces.RealOutput w annotation (
      Placement(transformation(extent = {{90, -70}, {110, -50}})));
  equation
    connect(drive.w, w) annotation (
      Line(points={{60,6},{60,-60},{100,-60}},                             color = {0, 0, 127}));
    connect(stimuli1.LoadTorque_Nm, drive.LoadTorque_Nm) annotation (
      Line(points={{-58.4,-12},{20,-12},{20,12},{36,12}},                                    color = {0, 0, 127}));
    connect(controller_sf_fmu.V, drive.V)
      annotation (Line(points={{-18,28},{36,28}}, color={0,0,127}));
    connect(stimuli1.w_desired, controller_sf_fmu.w_desired) annotation (Line(
          points={{-58.4,4},{-50,4},{-50,31.4},{-40.4,31.4}}, color={0,0,127}));
    connect(controller_sf_fmu.w, drive.w) annotation (Line(points={{-40.4,24.7},
            {-46,24.7},{-46,0},{60,0},{60,6}}, color={0,0,127}));
    annotation (
      Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})));
  end ControlledElectricDrive_ctrl_fmu_sl;

  model ControlledElectricDrive_ctrl_fmu_omc
    extends Modelica.Icons.Example;
    Drive drive annotation (
      Placement(transformation(extent = {{38, 0}, {78, 40}})));
    Stimuli stimuli1 annotation (
      Placement(transformation(extent = {{-80, -16}, {-50, 14}})));
    Modelica.Blocks.Interfaces.RealOutput w annotation (
      Placement(transformation(extent = {{90, -70}, {110, -50}})));
  MODELICA_Demo_Control_me_FMU mODELICA_Demo_Control_me_FMU1 annotation (
      Placement(visible = true, transformation(origin = {-8, 30}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  equation
    connect(mODELICA_Demo_Control_me_FMU1.w, drive.w) annotation (
      Line(points = {{-30, 44}, {-44, 44}, {-44, 0}, {58, 0}, {58, 8}, {58, 8}}, color = {0, 0, 127}));
    connect(stimuli1.w_desired, mODELICA_Demo_Control_me_FMU1.w_desired) annotation (
      Line(points = {{-48, 6}, {-40, 6}, {-40, 40}, {-30, 40}, {-30, 40}}, color = {0, 0, 127}));
    connect(mODELICA_Demo_Control_me_FMU1.V, drive.V) annotation (
      Line(points = {{14, 44}, {20, 44}, {20, 20}, {34, 20}, {34, 20}}, color = {0, 0, 127}));
    connect(drive.w, w) annotation (
      Line(points={{58,6},{58,6},{58,-60},{100,-60},{100,-60}},            color = {0, 0, 127}));
    connect(stimuli1.LoadTorque_Nm, drive.LoadTorque_Nm) annotation (
      Line(points={{-48.5,-8.5},{-32,-8.5},{-32,-20},{90,-20},{90,12},{34,12}},              color = {0, 0, 127}));
    annotation (
      Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})));
  end ControlledElectricDrive_ctrl_fmu_omc;

  annotation (
    uses(Modelica(version="4.0.0"), ModelicaServices(version="4.0.0")));
end MODELICA_Demo;

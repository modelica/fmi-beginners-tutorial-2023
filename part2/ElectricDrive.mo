within ;
model ElectricDrive
  Modelica.Blocks.Interfaces.RealInput v(unit="V") "Armature voltage"
                                         annotation (
    Placement(transformation(extent = {{-120, -10}, {-100, 10}}), iconTransformation(extent={{-140,
            -20},{-100,20}})));
  Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage annotation (
    Placement(transformation(extent = {{-20, 20}, {-40, 40}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Ground ground annotation (
    Placement(transformation(origin = {-50, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));
  Modelica.Mechanics.Rotational.Components.Inertia loadInertia(
    J=1,
    phi(fixed=true, start=0),
    w(fixed=true, start=0)) annotation (Placement(transformation(extent={{30,-10},
            {50,10}}, rotation=0)));
  Modelica.Mechanics.Rotational.Components.IdealGear idealGear(ratio = 10) annotation (
    Placement(transformation(extent = {{0, -10}, {20, 10}})));
  Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor annotation (
    Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin = {0, -30})));
  Modelica.Blocks.Interfaces.RealOutput w "Motor speed"
                                          annotation (
    Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin={0,-110}),    iconTransformation(extent = {{-10, -10}, {10, 10}}, rotation = 270, origin={0,-110})));
  Modelica.Mechanics.Rotational.Sources.Torque torque annotation (
    Placement(transformation(extent = {{10, -10}, {-10, 10}}, rotation = 0, origin = {70, 0})));
  Modelica.Blocks.Interfaces.RealInput tau(unit="N.m") "Load torque"
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={110,0}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=180,
        origin={120,0})));
  Modelica.Electrical.Machines.BasicMachines.DCMachines.DC_PermanentMagnet dcpm(VaNominal = dcpmData.VaNominal, IaNominal = dcpmData.IaNominal, wNominal = dcpmData.wNominal, TaNominal = dcpmData.TaNominal, Ra = dcpmData.Ra, TaRef = dcpmData.TaRef, La = dcpmData.La, Jr = dcpmData.Jr, useSupport = false, Js = dcpmData.Js, frictionParameters = dcpmData.frictionParameters, coreParameters = dcpmData.coreParameters, strayLoadParameters = dcpmData.strayLoadParameters, brushParameters = dcpmData.brushParameters, TaOperational = 293.15, alpha20a = dcpmData.alpha20a, phiMechanical(fixed = false), wMechanical(fixed = false), ia(fixed = true)) annotation (
    Placement(transformation(extent = {{-40, -10}, {-20, 10}})));
  parameter Modelica.Electrical.Machines.Utilities.ParameterRecords.DcPermanentMagnetData dcpmData(Jr = 0.001, wNominal = 149.22565104552, brushParameters(V = 0.7)) annotation (
    Placement(transformation(extent = {{-40, -40}, {-20, -20}})));
equation
  connect(ground.p, signalVoltage.n) annotation (
    Line(points={{-40,30},{-40,30}},      color = {0, 0, 255}, smooth = Smooth.None));
  connect(v, signalVoltage.v) annotation (
    Line(points={{-110,0},{-70,0},{-70,50},{-30,50},{-30,42}},            color = {0, 0, 127}, smooth = Smooth.None));
  connect(idealGear.flange_b, loadInertia.flange_a) annotation (Line(
      points={{20,0},{30,0}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(idealGear.flange_a, speedSensor.flange) annotation (
    Line(points = {{0, 0}, {1.77636e-15, 0}, {1.77636e-15, -20}}, color = {0, 0, 0}, smooth = Smooth.None));
  connect(speedSensor.w, w) annotation (
    Line(points={{-1.9984e-15,-41},{-1.9984e-15,-75.5},{0,-75.5},{0,-110}},
                                                             color = {0, 0, 127}, smooth = Smooth.None));
  connect(torque.flange, loadInertia.flange_b) annotation (Line(
      points={{60,0},{50,0}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(tau, torque.tau) annotation (Line(
      points={{110,-4.44089e-16},{94,-4.44089e-16},{94,0},{82,0}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(dcpm.flange, idealGear.flange_a) annotation (
    Line(points = {{-20, 0}, {0, 0}}, color = {0, 0, 0}));
  connect(dcpm.pin_an, ground.p) annotation (
    Line(points = {{-36, 10}, {-40, 10}, {-40, 30}}, color = {0, 0, 255}));
  connect(dcpm.pin_ap, signalVoltage.p) annotation (
    Line(points = {{-24, 10}, {-20, 10}, {-20, 30}}, color = {0, 0, 255}));
  annotation (
    Diagram(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}})),
    Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}), graphics={
                                Rectangle(
        extent={{-100,-100},{100,100}},
        lineColor={0,0,127},
        fillColor={255,255,255},
        fillPattern=FillPattern.Solid), Text(
        extent={{0,150},{0,110}},
        textColor={0,0,255},
          textString="%name"),
        Text(
          extent={{-90,10},{-90,-10}},
          textColor={0,0,0},
          horizontalAlignment=TextAlignment.Left,
          textString="V"),
        Text(
          extent={{90,10},{90,-10}},
          textColor={0,0,0},
          horizontalAlignment=TextAlignment.Right,
          textString="tau"),
        Text(
          extent={{0,-70},{0,-90}},
          textColor={0,0,0},
          textString="w")}),
    uses(Modelica(version="4.0.0")));
end ElectricDrive;

import "./arm-setpoints";
import "./arm-base";
import { FrcDashboard } from "@frc-web-components/fwc";

export default function addPlugin(dashboard: FrcDashboard) {
  dashboard.addElements(
    {
      "kwarqs-arm-setpoints": {
        dashboard: {
          displayName: "Arm Setpoints",
        },
        properties: {
          buttonselected: { type: "Number" },
        },
      },
      "kwarqs-arm-base": {
        dashboard: {
          displayName: "Arm Base",
        },
        properties: {
          angleMeasured: { type: "Number", attribute: "angle-measured" },
          telescopeLenMeasured: {
            type: "Number",
            attribute: "telescope-len-measured",
          },
          angleSetpoint: { type: "Number", attribute: "angle-setpoint" },
          telescopeLenSetpoint: {
            type: "Number",
            attribute: "telescope-len-setpoint",
          },
        },
      },
    },
    "My Elements"
  );
}

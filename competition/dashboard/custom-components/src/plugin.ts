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
      },
      "kwarqs-arm-base": {
        dashboard: {
          displayName: "Arm Base",
        },
        properties: {
          angle:{type:"Number", primary: true},
          telescopeLen:{type:"Number", attribute:"telescope-len"}
        }
      },
    },
    "My Elements"
  );
}

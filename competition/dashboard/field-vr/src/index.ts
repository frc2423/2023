import createDashboard from "@frc-web-components/fwc/lit/create-dashboard";
import "@frc-web-components/fwc/components";

console.log('!!!');

createDashboard({
  address: location.hostname,
  rootElement: "#root",
  render: ({ html, nt }) => {
    return html`
      <frc-field3d
        game="Evergreen"
        origin="red"
        background-color="black"
        enable-vr=""
      >
        <frc-field3d-object
          name="KitBot"
          translation="[4,3,0]"
          rotation="[0,0,0,0]"
        ></frc-field3d-object>
      </frc-field3d>
    `;
  },
});

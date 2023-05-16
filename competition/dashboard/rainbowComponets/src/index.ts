import createDashboard from "@frc-web-components/fwc/lit/create-dashboard";
import "@frc-web-components/fwc/components";
import "./rainbow-button"

console.log('!!!');

createDashboard({
  address: location.hostname,
  rootElement: "#root",
  render: ({ html, nt }) => {
    return html`
      <rainbow-button></rainbow-button>
    `;
  },
});

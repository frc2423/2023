import { html, css, LitElement } from "lit";
import { customElement, property } from "lit/decorators.js";
import getAssetUrl from "@frc-web-components/fwc/get-asset-url";

@customElement("kwarqs-arm-base")
export class ArmBase extends LitElement {
  static styles = css`
    :host {
      width: 500px;
      height: 500px;
      display: inline-block;
    }

    .container {
      position: relative;
      width: 100%;
      height: 100%;
    }

    .button {
      position: absolute;
      transform: translate(-50%, -50%);

      width: 15%;
      height: 15%;
      font-size: 30px;
      border-radius: 999px;
    }

    svg {
      width: 100%;
      height: 100%;
    }
  `;

  /**
   * The number of times the button has been clicked.
   */
  @property({ type: Number, reflect: true })
  count = 0;

  private onClick() {
    this.count++;
  }

  render() {
    const rect = this.getBoundingClientRect();

    const name = 'Amowee';

    // const sayHello = 'Hello ' + name + ' how are you?';
    const sayHello = `Hello ${name} how are you`;

    return html`
      <div class="container">
        <svg width=${rect.width} height=${rect.height}>
          <line x1="${rect.width / 2}" y1="${rect.height}" x2="${rect.width / 2}" y2="20" stroke="black" />
        </svg>
      </div>
    `;
  }
}

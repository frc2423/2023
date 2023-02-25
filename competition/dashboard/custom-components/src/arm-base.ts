import { html, css, LitElement } from "lit";
import { customElement, property } from "lit/decorators.js";
import getAssetUrl from "@frc-web-components/fwc/get-asset-url";

@customElement("kwarqs-arm-base")
export class ArmBase extends LitElement {
  @property({type:Number}) angle = 0
  @property({type:Number, attribute:"telescope-len"}) telescopeLen = 0
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
      overflow: visible;
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

  firstUpdated(): void {
    const observer = new ResizeObserver(() => this.requestUpdate());
    observer.observe(this);
  }

  Xpos(theta: number, x: number) {
    theta = (theta * (Math.PI/180)) * -1
    let xpos = x * Math.cos(theta)
    return xpos;
  }

  Ypos(theta: number, x: number) {
    theta = (theta * (Math.PI/180)) * -1
    let ypos = x * Math.sin(theta)
    return ypos;
  }

  render() {
    const rect = this.getBoundingClientRect();

    const name = 'Amowee';

    // const sayHello = 'Hello ' + name + ' how are you?';
    const sayHello = `Hello ${name} how are you`;

    let x2 = this.Xpos(this.angle, rect.height / 2) + (rect.width / 2)
    let y2 = this.Ypos(this.angle, rect.height / 2) + (rect.height)
    let x3 = this.Xpos(this.angle, rect.height / 2 + this.telescopeLen) + (rect.width / 2)
    let y3 = this.Ypos(this.angle, rect.height / 2 + this.telescopeLen) + (rect.height)

    return html`
      <div class="container">
        <svg width=${rect.width} height=${rect.height}>
          <line x1="${rect.width / 2}" y1="${rect.height}" x2="${x2}" y2="${y2}" stroke="orange" stroke-width="20" />
          <line x1="${x2}" y1="${y2}" x2="${x3}" y2="${y3}" stroke="yellow" stroke-width="14" />
          <rect x="${(rect.width / 2) - 150}" y="${rect.height + 150}" width="300" height="75" rx="15" fill="steelblue" />
          <polygon x="${rect.height}" points="150,100 150,100 200,200 100,200" fill="none" stroke="black" />
        </svg>
      </div>
    `;
  }
}

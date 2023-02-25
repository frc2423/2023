import { html, css, LitElement } from "lit";
import { customElement, property } from "lit/decorators.js";
import getAssetUrl from "@frc-web-components/fwc/get-asset-url";

@customElement("kwarqs-arm-base")
export class ArmBase extends LitElement {
  @property({ type: Number, attribute: "angle-measured" }) angleMeasured = 0;
  @property({ type: Number, attribute: "telescope-len-measured" }) telescopeLenMeasured = 0;
  @property({ type: Number, attribute: "angle-setpoint"}) angleSetpoint = 0;
  @property({ type: Number, attribute: "telescope-len-setpoint" }) telescopeLenSetpoint = 0;
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
    theta = theta * (Math.PI / 180) * -1;
    let xpos = x * Math.cos(theta);
    return xpos;
  }

  Ypos(theta: number, x: number) {
    theta = theta * (Math.PI / 180) * -1;
    let ypos = x * Math.sin(theta);
    return ypos;
  }

  render() {
    const rect = this.getBoundingClientRect();

    const name = "Amowee";

    // const sayHello = 'Hello ' + name + ' how are you?';
    const sayHello = `Hello ${name} how are you`;

    let x2 = this.Xpos(this.angleMeasured, rect.height / 2) + rect.width / 2;
    let y2 = this.Ypos(this.angleMeasured, rect.height / 2) + rect.height;
    let x3 =
      this.Xpos(this.angleMeasured, rect.height / 2 + this.telescopeLenMeasured) +
      rect.width / 2;
    let y3 =
      this.Ypos(this.angleMeasured, rect.height / 2 + this.telescopeLenMeasured) + rect.height;
    let x4 = this.Xpos(this.angleSetpoint, rect.height / 2) + rect.width / 2;
    let y4 = this.Ypos(this.angleSetpoint, rect.height / 2) + rect.height;
    let x5 =
      this.Xpos(this.angleSetpoint, rect.height / 2 + this.telescopeLenSetpoint) +
      rect.width / 2;
    let y5 =
      this.Ypos(this.angleSetpoint, rect.height / 2 + this.telescopeLenSetpoint) + rect.height;

    const points: [number, number][] = [
      [0, 0],
      [0, 100],
      [100, 100],
    ];

    // const pointsAtr = points
    //   .map(point => [point[0] + rect.width / 2, [point[1] + rect.height]])
    //   .map(point => point.join(' ')).join(',');

    const pointsAtr = `${rect.width / 2},${rect.height} ${rect.width / 2},${
      rect.height
    } 120,660 380,660`;

    return html`
      <div class="container">
        <svg width=${rect.width} height=${rect.height}>
          <polygon
            points=${pointsAtr}
            fill="none"
            stroke="purple"
            stroke-width="20"
          />
          <line
            x1="${rect.width / 2}"
            y1="${rect.height}"
            x2="${x2}"
            y2="${y2}"
            stroke="orange"
            stroke-width="20"
          />
          <line
            x1="${x2}"
            y1="${y2}"
            x2="${x3}"
            y2="${y3}"
            stroke="yellow"
            stroke-width="14"
          />
          <line
            x1="${rect.width / 2}"
            y1="${rect.height}"
            x2="${x4}"
            y2="${y4}"
            stroke="orange"
            stroke-width="20"
            style="opacity:0.25"
          />
          <line
            x1="${x4}"
            y1="${y4}"
            x2="${x5}"
            y2="${y5}"
            stroke="yellow"
            stroke-width="14"
            style="opacity:0.25"
          />
          <rect
            x="${rect.width / 2 - 150}"
            y="${rect.height + 150}"
            width="300"
            height="75"
            rx="15"
            fill="steelblue"
          />
          <text x="${rect.width / 2 - 75}" y="${rect.height + 200}" style="font-weight:bold; font-size:59px">
            2423
          </text>
        </svg>
      </div>
    `;
  }
}

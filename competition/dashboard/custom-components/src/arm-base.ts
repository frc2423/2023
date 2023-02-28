import { html, css, LitElement } from "lit";
import { customElement, property } from "lit/decorators.js";

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
    const triangleWidth = 200;
    const triangleHeight = 100;
    const blueBoxHeight = 75;
    const triangleX = rect.width / 2;
    const triangleY = rect.height - blueBoxHeight - triangleHeight;
    const armY = 500 - blueBoxHeight - triangleHeight;
      
    let x2 = this.Xpos(this.angleMeasured, armY / 2) + triangleX;
    let y2 = this.Ypos(this.angleMeasured, armY / 2) + triangleY;
    let x3 =
      this.Xpos(this.angleMeasured, armY / 2 + this.telescopeLenMeasured) +
      triangleX;
    let y3 =
      this.Ypos(this.angleMeasured, armY / 2 + this.telescopeLenMeasured) + triangleY;
    let x4 = this.Xpos(this.angleSetpoint, armY / 2) + triangleX;
    let y4 = this.Ypos(this.angleSetpoint, armY / 2) + triangleY;
    let x5 =
      this.Xpos(this.angleSetpoint, armY / 2 + this.telescopeLenSetpoint) +
      triangleX;
    let y5 =
      this.Ypos(this.angleSetpoint, armY / 2 + this.telescopeLenSetpoint) + triangleY;
      
      const pointsAtr = `${triangleX},${
        triangleY
      } ${triangleX - triangleWidth / 2}, ${triangleY + triangleHeight} ${triangleX + triangleWidth / 2}, ${triangleY + triangleHeight}`;
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
            x1="${triangleX}"
            y1="${triangleY}"
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
            x1="${triangleX}"
            y1="${triangleY}"
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
            y="${rect.height - blueBoxHeight}"
            width="300"
            height="${blueBoxHeight}"
            rx="15"
            fill="steelblue"
          />
          <text x="${rect.width / 2 - 75}" y="${rect.height - 20}" style="font-weight:bold; font-size:59px">
            2423
          </text>
        </svg>
      </div>
    `;
  }
}

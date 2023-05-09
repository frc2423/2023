import { html, css, LitElement } from "lit";
import { customElement, property } from "lit/decorators.js";
import "@frc-web-components/fwc/components";
import { NT4_Client } from "@frc-web-components/fwc/source-providers/nt4/NT4";

/**
 * An example element.
 *
 * @slot - This element has a slot
 * @csspart button - The button
 */
@customElement("field-vr")
export class MyElement extends LitElement {
  static styles = css`
    :host {
      display: block;
      border: solid 1px gray;
      padding: 16px;
      max-width: 800px;
    }
  `;

  /**
   * The name to say "Hello" to.
   */
  @property()
  name = "World";

  /**
   * The number of times the button has been clicked.
   */
  @property({ type: Number })
  count = 0;

  render() {
    return html`
      <frc-field3d game="Infinite Recharge" origin="red" background-color="black" enable-vr="">
        <frc-field3d-object name="KitBot" translation="[4,3,0]" rotation="[0,0,0,0]"></frc-field3d-object>
      </frc-field3d>
    `;
  }

  private _onClick() {
    this.count++;
  }

  foo(): string {
    return "foo";
  }
}



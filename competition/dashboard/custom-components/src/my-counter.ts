import { html, css, LitElement } from "lit";
import { customElement, property } from "lit/decorators.js";
import getAssetUrl from '@frc-web-components/fwc/get-asset-url';

@customElement("my-counter")
export class MyCounter extends LitElement {
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
    transform:translate(-50%,-50%);
    
    width:15%;
    height:15%;
    font-size:30px;
    border-radius:999px;
   }
   .position1 {
    left: 90%; top: 60%;
   }
   .position2 {
    left: 87%; top: 44.6%;
   }
   .position3 {
    left: 78.2%; top: 31.8%;
   }
   .position4 {
    left: 65.4%; top: 23%;
   }
   .position5 {
    left: 50%; top: 20%;
   }
   .position6 {
    left: 34.6%; top: 23%;
   }
   .position7 {
    left: 21.8%; top: 31.8%;
   }
   .position8 {
    left: 13%; top: 44.6%;
   }
   .position9 {
    left: 10%; top: 60%;
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
    return html`
    <div class="container">

      <button
        @click=${this.onClick}
        part="90degrees"
        style='background-image: url("${getAssetUrl("button-background.jpg")}" )'
        class="button position1"
        >
        a1
      </button>


      <button
        @click=${this.onClick}
        part="90degrees"
        style='background-image: url("${getAssetUrl("button-background.jpg")}" )'
        class="button position2"
        >
        a2
      </button>


      
      <button
        @click=${this.onClick}
        part="90degrees"
        style='background-image: url("${getAssetUrl("button-background.jpg")}" )'
        class="button position3"
        >
        a3
      </button>


      
      <button
        @click=${this.onClick}
        part="90degrees"
        style='background-image: url("${getAssetUrl("button-background.jpg")}" )'
        class="button position4"
        >
        a4
      </button>

      
      <button
        @click=${this.onClick}
        part="90degrees"
        style='background-image: url("${getAssetUrl("button-background.jpg")}" )'
        class="button position5"
        >
        a5
        </button>
      <button
        @click=${this.onClick}
        part="90degrees"
        style='background-image: url("${getAssetUrl("button-background.jpg")}" )'
        class="button position6"
        >
        a6
      </button>
        
      <button
        @click=${this.onClick}
        part="90degrees"
        style='background-image: url("${getAssetUrl("button-background.jpg")}" )'
        class="button position7"
        >
        a7
      </button>
      <button
        @click=${this.onClick}
        part="90degrees"
        style='background-image: url("${getAssetUrl("button-background.jpg")}" )'
        class="button position8"
        >
        a8
      </button>
      <button
        @click=${this.onClick}
        part="90degrees"
        style='background-image: url("${getAssetUrl("button-background.jpg")}" )'
        class="button position9"
        >
        a9
      </button>
    </div>
    `;
  }
}

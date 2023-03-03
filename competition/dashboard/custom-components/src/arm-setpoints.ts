import { html, css, LitElement } from "lit";
import { customElement, property } from "lit/decorators.js";
import getAssetUrl from '@frc-web-components/fwc/get-asset-url';

@customElement("kwarqs-arm-setpoints")
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
    font-size:20px;
    border-radius:999px;
   }
   .button-on {
    background: green !important;
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
  buttonselected = 1;

  private onClick(buttonnumber:number) {
    this.buttonselected = buttonnumber;
    if (this.buttonselected == 1) {
      html`<button style='background-color: #ff00ff'>`
    }
  }

  render() {
    return html`
    <div class="container">

      <button
        @click=${()=>this.onClick(1)}
        part = "90degrees"
        style='background-image: url("${getAssetUrl("button-background.jpg")}" )'
        class = '${`button position1 ${this.buttonselected == 1 ? "button-on" : ""}`}'
        >
        Floor
      </button>


      <button
        @click=${()=>this.onClick(2)}
        part="90degrees"
        style='background-image: url("${getAssetUrl("button-background.jpg")}" )'
        class = '${`button position2 ${this.buttonselected == 2 ? "button-on" : ""}`}'
        >
        Score<br>Mid
      </button>


      
      <button
        @click=${()=>this.onClick(3)}
        part="90degrees"
        style='background-image: url("${getAssetUrl("button-background.jpg")}" )'
        class='${`button position3 ${this.buttonselected == 3 ? "button-on" : ""}`}'
        >
        Score<br>High
      </button>


      
      <button
        @click=${()=>this.onClick(4)}
        part="90degrees"
        style='background-image: url("${getAssetUrl("button-background.jpg")}" )'
        class='${`button position4 ${this.buttonselected == 4 ? "button-on" : ""}`}'
        >
        HP
      </button>

      
      <button
        @click=${()=>this.onClick(5)}
        part="90degrees"
        style='background-image: url("${getAssetUrl("button-background.jpg")}" )'
        class='${`button position5 ${this.buttonselected == 5 ? "button-on" : ""}`}'
        >
        Up
        </button>
      <button
        @click=${()=>this.onClick(6)}
        part="90degrees"
        style='background-image: url("${getAssetUrl("button-background.jpg")}" )'
        class='${`button position6 ${this.buttonselected == 6 ? "button-on" : ""}`}'
        >
        HP
      </button>
        
      <button
        @click=${()=>this.onClick(7)}
        part="90degrees"
        style='background-image: url("${getAssetUrl("button-background.jpg")}" )'
        class='${`button position7 ${this.buttonselected == 7 ? "button-on" : ""}`}'
        >
        Score<br>High
      </button>
      <button
        @click=${()=>this.onClick(8)}
        part="90degrees"
        style='background-image: url("${getAssetUrl("button-background.jpg")}" )'
        class='${`button position8 ${this.buttonselected == 8 ? "button-on" : ""}`}'
        >
        Score<br>Mid
      </button>
      <button
        @click=${()=>this.onClick(9)}
        part="90degrees"
        style='background-image: url("${getAssetUrl("button-background.jpg")}" )'
        class='${`button position9 ${this.buttonselected == 9 ? "button-on" : ""}`}'
        >
        Floor
      </button>
    </div>
    <>
    `;
  }
}

import { html, css, LitElement } from 'lit'
import { customElement, property } from 'lit/decorators.js'

/**
 * An example element.
 *
 * @slot - This element has a slot
 * @csspart button - The button
 */
@customElement('rainbow-button')
export class RainbowButton extends LitElement {
  static styles = css`
    :host {
      display: block;
      padding: 16px;
      max-width: 800px;
    }
    :root {
      background-color: blue;
    }
  `

  /**
   * The name to say "Hello" to.
   */
  @property()
  name = 'World'

  /**
   * The number of times the button has been clicked.
   */
  @property({ type: Number })
  count = 0

  intervalId?: number;

  isRainbowOn = false;

  render() {
    return html`
      <button @click=${this._onClick} part="button" id="rainbow">
        Rainbow
      </button>
      <slot></slot>
    `
  }

  private _onClick() {
    this.count++
    if (this.count % 2 == 1) {
      document.body.classList.remove("body")
      document.body.classList.add("wrapper")

    }
    else {
      document.body.classList.remove("wrapper")
      document.body.classList.add("body")
    }
  }
  
//   private Rainbow() {
//     var colors = ["red", "orange", "yellow", "green", "blue", "indigo", "violet"];
//     var currentIndex = 0;
//     var toggleButton = document.getElementById("rainbow")!;

//     function changeBackground() {
//       var body = document.querySelector("body")!;
//       body.style.backgroundColor = colors[currentIndex];
//       currentIndex = (currentIndex + 1) % colors.length;

//     }
//     if (this.isRainbowOn) {
//       clearInterval(this.intervalId);
//       this.isRainbowOn = false;
//       toggleButton.textContent = "Toggle Rainbow Background";
//     } else {
//       this.intervalId = setInterval(changeBackground, 1000);
//       this.isRainbowOn = true;
//       toggleButton.textContent = "Stop Rainbow Background";

//     }
//   };
// }

// declare global {
//   interface HTMLElementTagNameMap {
//     'rainbow-button': RainbowButton
//   
}
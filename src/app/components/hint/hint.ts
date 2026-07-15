import { Component, Input } from '@angular/core';

/**
 * "?" icon with a hover tooltip. The popup uses position:fixed so it never
 * gets clipped by the debug hub's overflow scroll.
 */
@Component({
  selector: 'app-hint',
  standalone: true,
  template: `
    <span
      class="hint__icon"
      (mouseenter)="show($event)"
      (mouseleave)="visible = false"
      >?</span
    >
    @if (visible) {
      <div class="hint__pop" [class.hint__pop_above]="above" [style.left.px]="x" [style.top.px]="y">
        {{ text }}
      </div>
    }
  `,
  styleUrl: './hint.scss',
})
export class Hint {
  @Input({ required: true }) text = '';

  visible = false;
  above = false;
  x = 0;
  y = 0;

  show(e: MouseEvent): void {
    const r = (e.target as HTMLElement).getBoundingClientRect();
    this.x = Math.max(8, Math.min(r.left - 8, window.innerWidth - 268));
    // flip above the icon when near the bottom of the viewport
    this.above = r.bottom > window.innerHeight - 140;
    this.y = this.above ? r.top - 6 : r.bottom + 6;
    this.visible = true;
  }
}

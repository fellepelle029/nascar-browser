import { Component } from '@angular/core';
import { Router } from '@angular/router';

@Component({
  selector: 'app-home',
  imports: [],
  templateUrl: './home.html',
  styleUrl: './home.scss',
  standalone: true,
})
export class Home {
  constructor(private router: Router) {}

  navigateToRace() {
    try {
      this.router.navigate(['/race']);
    } catch (error) {
      console.error('Error navigating to race', error);
    }
  }

}

import { Routes } from '@angular/router';
import { Race } from './views/race/race';

export const routes: Routes = [
    {path: 'race', component: Race},
    {path: '**', redirectTo: 'race'},
];

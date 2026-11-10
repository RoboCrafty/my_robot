import { createRoot } from 'react-dom/client';
import '@fontsource-variable/geist';
import '@fontsource-variable/geist-mono';
import './styles.css';
import { App } from './App.jsx';
import { targetApi } from './scene/TargetGizmo.jsx';
import { useStore } from './store.js';

window.parol = { targetApi, useStore };   // debug handle for the browser console

createRoot(document.getElementById('root')).render(<App />);

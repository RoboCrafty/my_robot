import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';

// Served by server.py under /next while the new UI is being prototyped.
// `npm run dev` proxies the WebSocket and URDF assets to a running server.py.
export default defineConfig({
    base: '/next/',
    plugins: [react()],
    server: {
        port: 5173,
        proxy: {
            '/ws': { target: 'ws://localhost:8000', ws: true },
            '/assets': 'http://localhost:8000',
        },
    },
});

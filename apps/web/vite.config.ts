import react from '@vitejs/plugin-react';
import { defineConfig } from 'vitest/config';

// The FastAPI backend (apps/api) binds 127.0.0.1:8000 in local dev — see .env.example.
// Both REST and the WebSocket are proxied so the browser only ever talks to one origin
// and no CORS configuration is needed for `make dev`.
const API_ORIGIN = 'http://127.0.0.1:8000';

export default defineConfig({
  plugins: [react()],
  server: {
    port: 5173,
    strictPort: true,
    proxy: {
      '/api': {
        target: API_ORIGIN,
        changeOrigin: false,
      },
      '/ws': {
        target: API_ORIGIN,
        changeOrigin: false,
        ws: true,
      },
    },
  },
  build: {
    outDir: 'dist',
    sourcemap: true,
  },
  test: {
    environment: 'jsdom',
    globals: false,
    css: false,
    setupFiles: ['./src/test/setup.ts'],
    include: ['src/**/*.test.{ts,tsx}'],
  },
});

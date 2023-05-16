import { defineConfig } from 'vite'
import basicSsl from '@vitejs/plugin-basic-ssl';
import yargsHelpers from 'yargs/helpers';
import yargs from 'yargs/yargs';

const argv = yargs(yargsHelpers.hideBin(process.argv)).argv;

const plugins = [];

if (argv._.includes('--useHttps')) {
  plugins.push(basicSsl());
}

// https://vitejs.dev/config/
export default defineConfig({
  build: {
    lib: {
      entry: 'src/my-element.ts',
      formats: ['es']
    },
    rollupOptions: {
      external: /^lit/
    }
  },
  plugins,
})

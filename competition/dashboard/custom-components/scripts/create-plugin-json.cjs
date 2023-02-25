const fs = require("fs");
const path = require("path");
const packageJson = require('../package.json');

const pluginJsonPath = path.join(__dirname, "../plugin/plugin.json");

const pluginJson = {
  name: packageJson.name,
  description: packageJson.description,
  version: packageJson.version,
};

fs.writeFileSync(pluginJsonPath, JSON.stringify(pluginJson, null, 4));

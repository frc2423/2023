import './my-counter';
import createDashboard from '@frc-web-components/fwc';
import loadLocalStorage from '@frc-web-components/fwc/load-local-storage';
import addPlugin from './plugin';

const dashboard = createDashboard(document.body);
loadLocalStorage(dashboard);
addPlugin(dashboard);

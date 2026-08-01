'use strict';

let historyCapacity = 120;
let pollingIntervalMs = 1000;
const connectionHistory = [];
const metricHistory = new Map();
const graphColors = ['#3ddc97', '#7ec8ff', '#ffbf47', '#ff647c'];
let lastSuccess = 0;

const byId = (id) => document.getElementById(id);

function appendBounded(values, value) {
  values.push(value);
  while (values.length > historyCapacity) values.shift();
}

function flattenNumbers(value, prefix, output, depth = 0) {
  if (depth > 5 || output.size >= 48) return;
  if (typeof value === 'number' && Number.isFinite(value)) {
    output.set(prefix, value);
    return;
  }
  if (!value || typeof value !== 'object') return;
  Object.entries(value).forEach(([key, child]) => {
    const name = prefix ? `${prefix}.${key}` : key;
    flattenNumbers(child, name, output, depth + 1);
  });
}

function detailNumbers(dependency) {
  const output = new Map();
  const prefix = dependency.name || 'unknown';
  try {
    flattenNumbers(JSON.parse(dependency.detail), prefix, output);
  } catch (error) {
    const pattern = /([A-Za-z_][\w.-]*)\s*[=:]\s*(-?\d+(?:\.\d+)?)/g;
    for (const match of dependency.detail.matchAll(pattern)) {
      const value = Number(match[2]);
      if (Number.isFinite(value)) output.set(`${prefix}.${match[1]}`, value);
    }
  }
  return output;
}

function usefulMetric(name) {
  return /(voltage|soc|percent|velocity|tof|ultrasonic|depth|frequency|rate|temperature|latency)/i.test(name);
}

function renderConnectionHistory() {
  const canvas = byId('history');
  const context = canvas.getContext('2d');
  context.clearRect(0, 0, canvas.width, canvas.height);
  context.strokeStyle = '#3ddc97';
  context.lineWidth = 3;
  context.beginPath();
  connectionHistory.forEach((value, index) => {
    const x = connectionHistory.length === 1 ? 0 : (index / (connectionHistory.length - 1)) * canvas.width;
    const y = value ? 25 : canvas.height - 25;
    if (index === 0) context.moveTo(x, y);
    else context.lineTo(x, y);
  });
  context.stroke();
}

function renderMetricGraph(names) {
  const canvas = byId('numeric-history');
  const context = canvas.getContext('2d');
  context.clearRect(0, 0, canvas.width, canvas.height);
  const legend = byId('graph-legend');
  legend.replaceChildren();
  names.forEach((name, graphIndex) => {
    const values = metricHistory.get(name) || [];
    if (values.length === 0) return;
    const minimum = Math.min(...values);
    const maximum = Math.max(...values);
    const span = maximum - minimum;
    context.strokeStyle = graphColors[graphIndex];
    context.lineWidth = 2;
    context.beginPath();
    values.forEach((value, index) => {
      const x = values.length === 1 ? 0 : (index / (values.length - 1)) * canvas.width;
      const normalized = span === 0 ? 0.5 : (value - minimum) / span;
      const y = canvas.height - 15 - normalized * (canvas.height - 30);
      if (index === 0) context.moveTo(x, y);
      else context.lineTo(x, y);
    });
    context.stroke();
    const item = document.createElement('span');
    item.style.setProperty('--metric-color', graphColors[graphIndex]);
    item.textContent = `${name} (${minimum.toFixed(2)}–${maximum.toFixed(2)})`;
    legend.append(item);
  });
}

function renderMetrics(snapshot) {
  const current = new Map();
  (snapshot.dependencies || []).forEach((dependency) => {
    detailNumbers(dependency).forEach((value, name) => current.set(name, value));
  });
  current.forEach((value, name) => {
    if (!metricHistory.has(name)) metricHistory.set(name, []);
    appendBounded(metricHistory.get(name), value);
  });

  const metrics = byId('metrics');
  metrics.replaceChildren();
  const names = [...current.keys()].sort((left, right) => {
    const priority = Number(usefulMetric(right)) - Number(usefulMetric(left));
    return priority || left.localeCompare(right);
  });
  byId('metrics-empty').hidden = names.length > 0;
  names.slice(0, 16).forEach((name) => {
    const card = document.createElement('article');
    card.className = 'card fresh';
    const label = document.createElement('span');
    label.textContent = name;
    const value = document.createElement('strong');
    value.className = 'metric-value';
    value.textContent = Number(current.get(name)).toFixed(3).replace(/\.?0+$/, '');
    card.append(label, value);
    metrics.append(card);
  });
  const graphNames = names.filter(usefulMetric).slice(0, 4);
  renderMetricGraph(graphNames.length ? graphNames : names.slice(0, 4));
}

function render(snapshot) {
  lastSuccess = Date.now();
  byId('state').textContent = snapshot.state || 'unknown';
  byId('sequence').textContent = String(snapshot.sequence || 0);
  const connection = byId('connection');
  connection.textContent = snapshot.connected ? 'Connected' : 'Disconnected';
  connection.className = `badge ${snapshot.connected ? 'fresh' : 'stale'}`;
  const dependencies = byId('dependencies');
  dependencies.replaceChildren();
  (snapshot.dependencies || []).forEach((dependency) => {
    const card = document.createElement('article');
    card.className = `card ${dependency.state}`;
    const name = document.createElement('strong');
    name.textContent = dependency.name;
    const state = document.createElement('span');
    state.textContent = `${dependency.state} · ${dependency.age_ms < 0 ? '—' : `${dependency.age_ms} ms`}`;
    const detail = document.createElement('span');
    detail.textContent = dependency.detail;
    card.append(name, state, detail);
    dependencies.append(card);
  });
  const alerts = byId('alerts');
  alerts.replaceChildren();
  const activeAlerts = snapshot.alerts?.length ? snapshot.alerts : ['None'];
  activeAlerts.forEach((alert) => {
    const item = document.createElement('li');
    item.textContent = alert;
    alerts.append(item);
  });
  appendBounded(connectionHistory, Boolean(snapshot.connected));
  renderConnectionHistory();
  renderMetrics(snapshot);
}

async function poll() {
  try {
    const response = await fetch('/api/telemetry', {cache: 'no-store'});
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    render(await response.json());
  } catch (error) {
    const connection = byId('connection');
    connection.textContent = 'Connection lost';
    connection.className = 'badge stale';
  }
  const age = lastSuccess ? Math.round((Date.now() - lastSuccess) / 1000) : null;
  byId('age').textContent = age === null ? '—' : `${age} s`;
  setTimeout(poll, pollingIntervalMs);
}

async function start() {
  try {
    const response = await fetch('/api/config', {cache: 'no-store'});
    if (response.ok) {
      const config = await response.json();
      if (Number.isInteger(config.history_capacity)) historyCapacity = config.history_capacity;
      if (Number.isInteger(config.polling_interval_ms)) pollingIntervalMs = config.polling_interval_ms;
    }
  } catch (error) {
    // Safe defaults above keep the dashboard operational without configuration data.
  }
  poll();
}

start();

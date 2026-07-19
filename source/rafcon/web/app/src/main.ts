import { createApp } from 'vue'
import { createPinia } from 'pinia'
import App from './App.vue'
import { startSocket } from './services/socket'
import { windowMode, windowTitle } from './services/windowMode'
import { useStateMachinesStore } from './stores/statemachines'
import './style.css'

document.title = windowTitle(windowMode)

const app = createApp(App)
const pinia = createPinia()
app.use(pinia)

// pin before the socket connects so the first SYNC already respects it
if (windowMode.kind === 'sm') {
  useStateMachinesStore(pinia).pin(windowMode.smId)
}

app.mount('#app')

startSocket()

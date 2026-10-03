import * as THREE from 'three';
import { useEffect } from 'react';
import { Canvas } from '@react-three/fiber';
import { AnimatePresence, MotionConfig } from 'motion/react';
import { Toaster, toast } from 'sonner';
import { connect, hardStop, useStore } from './store.js';
import { Scene, THEMES } from './scene/Scene.jsx';
import { TopBar } from './hud/TopBar.jsx';
import { Dock } from './hud/Dock.jsx';
import { Readout } from './hud/Readout.jsx';
import { TargetCard } from './hud/TargetCard.jsx';
import { JogHud } from './hud/JogHud.jsx';
import { Panel } from './hud/Panel.jsx';

const typing = (t) => t && (t.tagName === 'INPUT' || t.tagName === 'TEXTAREA' || t.tagName === 'SELECT');

export function App() {
    const mode = useStore((s) => s.mode);
    const theme = useStore((s) => s.prefs.theme);
    const quality = useStore((s) => s.prefs.quality);
    const panel = useStore((s) => s.prefs.panel);
    const cinematic = quality === 'cinematic';

    useEffect(() => { connect(); }, []);

    useEffect(() => {
        document.documentElement.dataset.theme = theme;
        document.querySelector('meta[name="theme-color"]')?.setAttribute('content', THEMES[theme].bg);
    }, [theme]);

    useEffect(() => {
        const onKey = (e) => {
            if (typing(e.target)) return;
            if (e.code === 'Space') {
                // Hard stop from anywhere except while typing.
                e.preventDefault();
                hardStop();
                toast.error('Stopped', { id: 'stop' });
            } else if (e.key === 'Escape') {
                useStore.setState({ mode: 'orbit' });
            } else if (e.key === 't' || e.key === 'T') {
                useStore.setState((s) => ({ mode: s.mode === 'tcp' ? 'orbit' : 'tcp' }));
            }
        };
        addEventListener('keydown', onKey);
        return () => removeEventListener('keydown', onKey);
    }, []);

    return (
        <MotionConfig reducedMotion="user" transition={{ type: 'spring', duration: 0.35, bounce: 0.12 }}>
            <div className={`app ${panel ? 'with-panel' : ''} mode-${mode}`}>
                <div className="stage">
                    {/* Render on demand: a still arm should cost no GPU at all. */}
                    <Canvas
                        key={quality}
                        frameloop="demand"
                        shadows={!cinematic}
                        dpr={cinematic ? [1, 2] : [1, 1.5]}
                        gl={{ antialias: !cinematic, powerPreference: 'high-performance' }}
                        camera={{ fov: 40, near: 0.01, far: 40, position: [1, 0.8, 1] }}
                        onCreated={({ gl }) => { if (!cinematic) gl.toneMapping = THREE.AgXToneMapping; }}
                    >
                        <Scene />
                    </Canvas>
                    {!cinematic && <div className="vignette" aria-hidden />}
                </div>

                <TopBar />
                <Readout />
                <AnimatePresence>{mode === 'tcp' && <TargetCard key="tcp" />}</AnimatePresence>
                <JogHud />
                <Dock />
                <Panel />
                <Toaster theme={theme} position="top-center" toastOptions={{ className: 'toast' }} />
            </div>
        </MotionConfig>
    );
}

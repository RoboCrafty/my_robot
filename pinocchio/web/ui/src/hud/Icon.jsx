const PATHS = {
    orbit: <><circle cx="12" cy="12" r="3" /><path d="M3 12c0-2.5 4-4.5 9-4.5s9 2 9 4.5-4 4.5-9 4.5" /><path d="m16 15 2.5 1.5L16 19" /></>,
    target: <><circle cx="12" cy="12" r="7" /><circle cx="12" cy="12" r="2" /><path d="M12 2v3M12 19v3M2 12h3M19 12h3" /></>,
    trail: <><path d="M4 18c3 0 4-3 7-6s5-5 9-5" /><circle cx="20" cy="7" r="1.6" /></>,
    erase: <><path d="M5 19h14" /><path d="m7 15 8-8 3 3-8 8H8z" /></>,
    curve: <><path d="M4 18C6 8 14 4 20 6" /><circle cx="4" cy="18" r="1.6" /><circle cx="20" cy="6" r="1.6" /></>,
    line: <><path d="M5 18 19 6" /><circle cx="5" cy="18" r="1.6" /><circle cx="19" cy="6" r="1.6" /></>,
    reset: <><path d="M4 12a8 8 0 1 0 2.3-5.6" /><path d="M4 4v4h4" /></>,
    close: <path d="M6 6l12 12M18 6 6 18" />,
    play: <path d="M7 5v14l11-7z" />,
    pause: <path d="M8 5v14M16 5v14" />,
    step: <><path d="M6 5v14l9-7z" /><path d="M18 5v14" /></>,
    square: <rect x="6" y="6" width="12" height="12" rx="1.5" />,
    plus: <path d="M12 5v14M5 12h14" />,
    minus: <path d="M5 12h14" />,
    up: <path d="m6 14 6-6 6 6" />,
    down: <path d="m6 10 6 6 6-6" />,
    indent: <><path d="M4 6h16M10 12h10M4 18h16" /><path d="m4 9 3 3-3 3" /></>,
    outdent: <><path d="M4 6h16M10 12h10M4 18h16" /><path d="m7 9-3 3 3 3" /></>,
    copy: <><rect x="8" y="8" width="12" height="12" rx="2" /><path d="M16 8V6a2 2 0 0 0-2-2H6a2 2 0 0 0-2 2v8a2 2 0 0 0 2 2h2" /></>,
    trash: <><path d="M4 7h16M10 11v6M14 11v6" /><path d="M6 7l1 12a2 2 0 0 0 2 2h6a2 2 0 0 0 2-2l1-12M9 7V4h6v3" /></>,
    sun: <><circle cx="12" cy="12" r="4" /><path d="M12 2v2M12 20v2M4.9 4.9l1.4 1.4M17.7 17.7l1.4 1.4M2 12h2M20 12h2M4.9 19.1l1.4-1.4M17.7 6.3l1.4-1.4" /></>,
    moon: <path d="M20 14.5A8 8 0 0 1 9.5 4a8 8 0 1 0 10.5 10.5z" />,
    panel: <><rect x="3" y="4" width="18" height="16" rx="3" /><path d="M14 4v16" /></>,
    home: <><path d="M4 11 12 4l8 7" /><path d="M6 10v9h12v-9" /></>,
    sync: <><path d="M20 12a8 8 0 0 1-14 5.3M4 12a8 8 0 0 1 14-5.3" /><path d="M18 3v4h-4M6 21v-4h4" /></>,
    ready: <><path d="M5 19V9l7-5 7 5v10" /><path d="M12 19v-6" /></>,
    pin: <><path d="M12 21s-6-5.7-6-11a6 6 0 0 1 12 0c0 5.3-6 11-6 11z" /><circle cx="12" cy="10" r="2" /></>,
    edit: <path d="M4 20h4L19 9l-4-4L4 16zM13.5 6.5l4 4" />,
    grip: <><path d="M8 4v6a4 4 0 0 0 8 0V4" /><path d="M12 14v6" /></>,
    bolt: <path d="M13 2 4 14h7l-1 8 9-12h-7z" />,
    sliders: <><path d="M4 6h10M18 6h2M4 12h4M12 12h8M4 18h12M20 18h0" /><circle cx="16" cy="6" r="2" /><circle cx="10" cy="12" r="2" /><circle cx="18" cy="18" r="2" /></>,
    list: <><path d="M9 6h11M9 12h11M9 18h11" /><circle cx="4.5" cy="6" r="1" /><circle cx="4.5" cy="12" r="1" /><circle cx="4.5" cy="18" r="1" /></>,
    joints: <><circle cx="6" cy="18" r="2" /><circle cx="12" cy="10" r="2" /><circle cx="18" cy="6" r="2" /><path d="m7.4 16.6 3.2-5.2M13.7 9l2.6-1.9" /></>,
    gear: <><circle cx="12" cy="12" r="3" /><path d="M12 2v3M12 19v3M4.9 4.9 7 7M17 17l2.1 2.1M2 12h3M19 12h3M4.9 19.1 7 17M17 7l2.1-2.1" /></>,
};

export function Icon({ name, size = 16 }) {
    return (
        <svg width={size} height={size} viewBox="0 0 24 24" fill="none" stroke="currentColor"
            strokeWidth="1.7" strokeLinecap="round" strokeLinejoin="round" aria-hidden>
            {PATHS[name]}
        </svg>
    );
}

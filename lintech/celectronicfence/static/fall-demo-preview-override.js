(function () {
    const DEMO_VIDEO_MAP = {
        '跌倒检测': '/fall-demo-loop.mp4',
        'fall_detection': '/fall-demo-loop.mp4',
        '打架检测': '/fight-demo-loop.mp4',
        'fight_detection': '/fight-demo-loop.mp4'
    };
    const originalUpdateGridSlot = window.updateGridSlot;
    const originalLoadStreamTaskVideo = window.loadStreamTaskVideo;

    function isDemoModeEnabled(value) {
        return value === true || value === 1 || value === '1' || value === 'true';
    }

    function getDemoModeMap() {
        try {
            const raw = window.localStorage.getItem('cefTaskDemoModeMap');
            const parsed = raw ? JSON.parse(raw) : {};
            return parsed && typeof parsed === 'object' ? parsed : {};
        } catch (error) {
            console.warn('读取演示模式配置失败:', error);
            return {};
        }
    }

    function isTaskDemoModeEnabled(task) {
        if (!task) {
            return false;
        }

        if (Object.prototype.hasOwnProperty.call(task, 'demoMode')) {
            return isDemoModeEnabled(task.demoMode);
        }

        const map = getDemoModeMap();
        return isDemoModeEnabled(map[String(task.id)]);
    }

    function getDemoVideoPath(task) {
        if (!task || !isTaskDemoModeEnabled(task)) {
            return '';
        }

        const algorithm = String((task && task.algorithm) || '').trim();
        return DEMO_VIDEO_MAP[algorithm] || '';
    }

    function buildDemoSrcdoc(videoPath) {
        const videoUrl = window.location.origin + videoPath;
        return `
<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<style>
html, body {
    margin: 0;
    width: 100%;
    height: 100%;
    overflow: hidden;
    background: #000;
}
body {
    display: flex;
    align-items: center;
    justify-content: center;
}
video {
    width: 100%;
    height: 100%;
    object-fit: contain;
    background: #000;
}
</style>
</head>
<body>
<video autoplay muted loop playsinline preload="auto">
<source src="${videoUrl}" type="video/mp4">
</video>
</body>
</html>`.trim();
    }

    function applyDemoPreview(iframe, videoPath) {
        if (!iframe || !videoPath) {
            return;
        }

        const nextSrcdoc = buildDemoSrcdoc(videoPath);
        if (iframe.srcdoc !== nextSrcdoc) {
            iframe.srcdoc = nextSrcdoc;
        }
    }

    function clearDemoPreview(iframe) {
        if (!iframe) {
            return;
        }

        if (iframe.hasAttribute('srcdoc')) {
            iframe.removeAttribute('srcdoc');
        }
    }

    window.updateGridSlot = function (index, item) {
        const iframe = document.getElementById(`grid-iframe-${index}`);
        const videoPath = item ? getDemoVideoPath(item.task) : '';

        if (!item || !videoPath) {
            clearDemoPreview(iframe);
            if (typeof originalUpdateGridSlot === 'function') {
                return originalUpdateGridSlot(index, item);
            }
            return;
        }

        const statusDot = document.querySelector(`#grid-${index} .stream-grid-status`);
        const emptyMsg = document.querySelector(`#grid-${index} .stream-grid-empty`);
        const label = document.querySelector(`#grid-${index} .stream-grid-label`);
        const task = item.task;
        const channel = item.channel || null;

        applyDemoPreview(iframe, videoPath);

        if (label) {
            label.textContent = channel && channel.name
                ? `${task.taskNumber || ('任务' + task.id)} - ${channel.name}`
                : (task.taskNumber || ('任务' + task.id));
        }

        if (statusDot) {
            statusDot.classList.remove('offline');
            statusDot.classList.add('online');
        }

        if (emptyMsg) {
            emptyMsg.style.display = 'none';
        }
    };

    window.loadStreamTaskVideo = function (task) {
        const iframe = document.getElementById('stream-iframe-0');
        const videoPath = getDemoVideoPath(task);

        if (!videoPath) {
            clearDemoPreview(iframe);
            if (typeof originalLoadStreamTaskVideo === 'function') {
                return originalLoadStreamTaskVideo(task);
            }
            return;
        }

        applyDemoPreview(iframe, videoPath);
        console.log((task && task.algorithm ? task.algorithm : '任务') + '预览已切换为循环演示视频');
    };
})();

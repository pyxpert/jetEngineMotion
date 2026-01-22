// 游戏配置
const GRID_SIZE = 20;
const CANVAS_WIDTH = 540;
const CANVAS_HEIGHT = 540;
const GRID_WIDTH = CANVAS_WIDTH / GRID_SIZE;
const GRID_HEIGHT = CANVAS_HEIGHT / GRID_SIZE;

// 游戏状态
let canvas, ctx;
let snake = [];
let food = {};
let direction = { x: 1, y: 0 };
let nextDirection = { x: 1, y: 0 };
let score = 0;
let highScore = 0;
let gameRunning = false;
let gamePaused = false;
let gameLoop = null;

// DOM元素
const gameCanvas = document.getElementById('gameCanvas');
const gameOverlay = document.getElementById('gameOverlay');
const overlayTitle = document.getElementById('overlayTitle');
const overlayMessage = document.getElementById('overlayMessage');
const scoreElement = document.getElementById('score');
const highScoreElement = document.getElementById('highScore');

// 方向按钮
const upBtn = document.getElementById('upBtn');
const downBtn = document.getElementById('downBtn');
const leftBtn = document.getElementById('leftBtn');
const rightBtn = document.getElementById('rightBtn');

// 初始化
function init() {
    canvas = gameCanvas;
    ctx = canvas.getContext('2d');
    canvas.width = CANVAS_WIDTH;
    canvas.height = CANVAS_HEIGHT;
    
    // 加载最高分
    highScore = parseInt(localStorage.getItem('snakeHighScore') || '0');
    highScoreElement.textContent = highScore;
    
    // 绑定事件
    bindEvents();
    
    // 初始化游戏
    resetGame();
}

// 绑定事件
function bindEvents() {
    // 键盘控制
    document.addEventListener('keydown', handleKeyPress);
    
    // 触摸/鼠标控制按钮
    upBtn.addEventListener('click', () => changeDirection(0, -1));
    downBtn.addEventListener('click', () => changeDirection(0, 1));
    leftBtn.addEventListener('click', () => changeDirection(-1, 0));
    rightBtn.addEventListener('click', () => changeDirection(1, 0));
    
    // 触摸控制（移动端）
    let touchStartX = 0;
    let touchStartY = 0;
    
    canvas.addEventListener('touchstart', (e) => {
        e.preventDefault();
        touchStartX = e.touches[0].clientX;
        touchStartY = e.touches[0].clientY;
    });
    
    canvas.addEventListener('touchend', (e) => {
        e.preventDefault();
        if (!touchStartX || !touchStartY) return;
        
        const touchEndX = e.changedTouches[0].clientX;
        const touchEndY = e.changedTouches[0].clientY;
        
        const diffX = touchStartX - touchEndX;
        const diffY = touchStartY - touchEndY;
        
        if (Math.abs(diffX) > Math.abs(diffY)) {
            // 水平滑动
            if (diffX > 0) {
                changeDirection(-1, 0); // 左
            } else {
                changeDirection(1, 0); // 右
            }
        } else {
            // 垂直滑动
            if (diffY > 0) {
                changeDirection(0, -1); // 上
            } else {
                changeDirection(0, 1); // 下
            }
        }
        
        touchStartX = 0;
        touchStartY = 0;
    });
}

// 处理按键
function handleKeyPress(e) {
    if (e.key === ' ' || e.key === 'Space') {
        e.preventDefault();
        if (!gameRunning) {
            startGame();
        } else {
            togglePause();
        }
        return;
    }
    
    if (!gameRunning || gamePaused) return;
    
    switch(e.key) {
        case 'ArrowUp':
        case 'w':
        case 'W':
            e.preventDefault();
            changeDirection(0, -1);
            break;
        case 'ArrowDown':
        case 's':
        case 'S':
            e.preventDefault();
            changeDirection(0, 1);
            break;
        case 'ArrowLeft':
        case 'a':
        case 'A':
            e.preventDefault();
            changeDirection(-1, 0);
            break;
        case 'ArrowRight':
        case 'd':
        case 'D':
            e.preventDefault();
            changeDirection(1, 0);
            break;
    }
}

// 改变方向
function changeDirection(x, y) {
    // 防止反向移动
    if (direction.x === -x && direction.y === -y) return;
    nextDirection = { x, y };
}

// 重置游戏
function resetGame() {
    snake = [
        { x: 10, y: 10 },
        { x: 9, y: 10 },
        { x: 8, y: 10 }
    ];
    direction = { x: 1, y: 0 };
    nextDirection = { x: 1, y: 0 };
    score = 0;
    gameRunning = false;
    gamePaused = false;
    scoreElement.textContent = score;
    generateFood();
    draw();
    showOverlay('贪吃蛇游戏', '按空格键开始游戏');
}

// 开始游戏
function startGame() {
    gameRunning = true;
    gamePaused = false;
    hideOverlay();
    if (gameLoop) clearInterval(gameLoop);
    gameLoop = setInterval(update, 150);
}

// 暂停/继续
function togglePause() {
    if (!gameRunning) return;
    
    gamePaused = !gamePaused;
    if (gamePaused) {
        clearInterval(gameLoop);
        showOverlay('游戏暂停', '按空格键继续');
    } else {
        hideOverlay();
        gameLoop = setInterval(update, 150);
    }
}

// 生成食物
function generateFood() {
    do {
        food = {
            x: Math.floor(Math.random() * GRID_WIDTH),
            y: Math.floor(Math.random() * GRID_HEIGHT)
        };
    } while (snake.some(segment => segment.x === food.x && segment.y === food.y));
}

// 更新游戏状态
function update() {
    if (!gameRunning || gamePaused) return;
    
    direction = { ...nextDirection };
    
    // 移动蛇头
    const head = {
        x: snake[0].x + direction.x,
        y: snake[0].y + direction.y
    };
    
    // 检查边界碰撞
    if (head.x < 0 || head.x >= GRID_WIDTH || head.y < 0 || head.y >= GRID_HEIGHT) {
        gameOver();
        return;
    }
    
    // 检查自身碰撞
    if (snake.some(segment => segment.x === head.x && segment.y === head.y)) {
        gameOver();
        return;
    }
    
    snake.unshift(head);
    
    // 检查是否吃到食物
    if (head.x === food.x && head.y === food.y) {
        score += 10;
        scoreElement.textContent = score;
        
        if (score > highScore) {
            highScore = score;
            highScoreElement.textContent = highScore;
            localStorage.setItem('snakeHighScore', highScore);
        }
        
        generateFood();
    } else {
        snake.pop();
    }
    
    draw();
}

// 绘制游戏
function draw() {
    // 清空画布
    ctx.fillStyle = '#2c3e50';
    ctx.fillRect(0, 0, CANVAS_WIDTH, CANVAS_HEIGHT);
    
    // 绘制网格
    ctx.strokeStyle = '#34495e';
    ctx.lineWidth = 1;
    for (let i = 0; i <= GRID_WIDTH; i++) {
        ctx.beginPath();
        ctx.moveTo(i * GRID_SIZE, 0);
        ctx.lineTo(i * GRID_SIZE, CANVAS_HEIGHT);
        ctx.stroke();
    }
    for (let i = 0; i <= GRID_HEIGHT; i++) {
        ctx.beginPath();
        ctx.moveTo(0, i * GRID_SIZE);
        ctx.lineTo(CANVAS_WIDTH, i * GRID_SIZE);
        ctx.stroke();
    }
    
    // 绘制食物
    ctx.fillStyle = '#e74c3c';
    ctx.fillRect(food.x * GRID_SIZE + 2, food.y * GRID_SIZE + 2, GRID_SIZE - 4, GRID_SIZE - 4);
    ctx.fillStyle = '#c0392b';
    ctx.fillRect(food.x * GRID_SIZE + 4, food.y * GRID_SIZE + 4, GRID_SIZE - 8, GRID_SIZE - 8);
    
    // 绘制蛇
    snake.forEach((segment, index) => {
        if (index === 0) {
            // 蛇头
            ctx.fillStyle = '#27ae60';
            ctx.fillRect(segment.x * GRID_SIZE + 1, segment.y * GRID_SIZE + 1, GRID_SIZE - 2, GRID_SIZE - 2);
            ctx.fillStyle = '#2ecc71';
            ctx.fillRect(segment.x * GRID_SIZE + 3, segment.y * GRID_SIZE + 3, GRID_SIZE - 6, GRID_SIZE - 6);
        } else {
            // 蛇身
            ctx.fillStyle = '#3498db';
            ctx.fillRect(segment.x * GRID_SIZE + 2, segment.y * GRID_SIZE + 2, GRID_SIZE - 4, GRID_SIZE - 4);
            ctx.fillStyle = '#2980b9';
            ctx.fillRect(segment.x * GRID_SIZE + 4, segment.y * GRID_SIZE + 4, GRID_SIZE - 8, GRID_SIZE - 8);
        }
    });
}

// 游戏结束
function gameOver() {
    gameRunning = false;
    clearInterval(gameLoop);
    showOverlay('游戏结束', `得分: ${score}<br>按空格键重新开始`);
}

// 显示遮罩层
function showOverlay(title, message) {
    overlayTitle.textContent = title;
    overlayMessage.innerHTML = message;
    gameOverlay.classList.remove('hidden');
}

// 隐藏遮罩层
function hideOverlay() {
    gameOverlay.classList.add('hidden');
}

// 初始化游戏
init();

import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/hackathon-book/ur/docs',
    component: ComponentCreator('/hackathon-book/ur/docs', '877'),
    routes: [
      {
        path: '/hackathon-book/ur/docs',
        component: ComponentCreator('/hackathon-book/ur/docs', '589'),
        routes: [
          {
            path: '/hackathon-book/ur/docs',
            component: ComponentCreator('/hackathon-book/ur/docs', '314'),
            routes: [
              {
                path: '/hackathon-book/ur/docs/docs',
                component: ComponentCreator('/hackathon-book/ur/docs/docs', 'f46'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/ur/docs/module-1/',
                component: ComponentCreator('/hackathon-book/ur/docs/module-1/', 'bfd'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/ur/docs/module-1/communication',
                component: ComponentCreator('/hackathon-book/ur/docs/module-1/communication', '6a8'),
                exact: true
              },
              {
                path: '/hackathon-book/ur/docs/module-1/I18N_WORKFLOW',
                component: ComponentCreator('/hackathon-book/ur/docs/module-1/I18N_WORKFLOW', '632'),
                exact: true
              },
              {
                path: '/hackathon-book/ur/docs/module-1/intro',
                component: ComponentCreator('/hackathon-book/ur/docs/module-1/intro', 'c8a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/hackathon-book/ur/docs/module-1/python-agents',
                component: ComponentCreator('/hackathon-book/ur/docs/module-1/python-agents', 'ad6'),
                exact: true
              },
              {
                path: '/hackathon-book/ur/docs/module-1/VERIFICATION_LOG',
                component: ComponentCreator('/hackathon-book/ur/docs/module-1/VERIFICATION_LOG', '9d8'),
                exact: true
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
